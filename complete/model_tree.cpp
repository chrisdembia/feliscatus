
#include "FelisCatusModeling.h"

using OpenSim::CustomJoint;
using OpenSim::PinJoint;
using OpenSim::SpatialTransform;
using OpenSim::WeldJoint;

/**
 * This class allows for the creation of a "tree" of cat models, starting with
 * a "zero- degree-of-freedom" (DOF) base case and adding complexity (i.e.,
 * DOFs) iteratively. Each cat model consists of two cylindrical
 * segments connected by a three-DOF (back flexion or 'hunch', spinal
 * twist, and side bending or 'wag') joint. Each model restricts the
 * DOFs of this joint to a different degree. In addition, some models
 * includes legs; some are rigid and normal to the cat's underside, while
 * others are "retractable," allowing the cat to change the effective
 * moment of inertia of its two halves. Any of these models can be used in
 * input to the 'optimize' executable, though it may be necessary to set some
 * of the objective weights to 0 depending on the model used (i.e., can't use
 * an objective that depends on legs if the model does not have legs).
 * */
class FelisCatusDembiaSketch : public FelisCatusModeling
{
public:

	enum LegsType {NoLegs, Rigid, RetractBack, Retract};

    enum TailType {NoTail, FixedPerch, FreePerch};

    // member functions to allow for stepwise model creation
	// NOTE: each 'add' function takes care of actuators if necessary
	FelisCatusDembiaSketch(string modelName, string fileName,
		bool canTwist, bool canHunch, bool canWag,
        LegsType whichLegs, TailType whichTail);
	void createBaseCase();
    void addTwist();
	void addHunch();
	void addWag();
	void addRigidLegs();
	void addRetractLegs(bool frontLegsRetract=true);
    void addTail(TailType whichTail);

    /**
     * Adds a coordinate actuator to the model for the given coordinate name.
     * The min and max control are set from the _maxTorque member variable.
     * */
    void addCoordinateActuator(string coordinateName);

private:

	static const int _maxTorque = 20; // N-m
};

int main(int argc, char *argv[])
{
    vector<bool> bothOptions;
    bothOptions.push_back(false);
    bothOptions.push_back(true);

    vector<bool> canTwist = bothOptions;
    vector<bool> canHunch = bothOptions;
    vector<bool> canWag = bothOptions;

    vector<FelisCatusDembiaSketch::LegsType> whichLegs;
    whichLegs.push_back(FelisCatusDembiaSketch::NoLegs);
    whichLegs.push_back(FelisCatusDembiaSketch::Rigid);
    whichLegs.push_back(FelisCatusDembiaSketch::RetractBack);
    whichLegs.push_back(FelisCatusDembiaSketch::Retract);

    vector<FelisCatusDembiaSketch::TailType> whichTail;
    whichTail.push_back(FelisCatusDembiaSketch::NoTail);
    whichTail.push_back(FelisCatusDembiaSketch::FixedPerch);
    whichTail.push_back(FelisCatusDembiaSketch::FreePerch);

    for (unsigned int it = 0; it < canTwist.size(); it++)
    {
        for (unsigned int ih = 0; ih < canHunch.size(); ih++)
        {
            for (unsigned int iw = 0; iw < canWag.size(); iw++)
            {
                for (int iL = 0; iL < whichLegs.size(); iL++)
                {
                    for (int iT = 0; iT < whichTail.size(); iT++)
                    {
                        string modifier;
                        modifier += canTwist[it] ? "Twist" : "";
                        modifier += canHunch[ih] ? "Hunch" : "";
                        modifier += canWag[iw] ? "Wag" : "";

                        switch (whichLegs[iL])
                        {
                            case FelisCatusDembiaSketch::Rigid:
                                modifier += "RigidLegs"; break;
                            case FelisCatusDembiaSketch::RetractBack:
                                modifier += "RetractBackLeg"; break;
                            case FelisCatusDembiaSketch::Retract:
                                modifier += "RetractLegs"; break;
                        }
                        switch (whichTail[iT])
                        {
                            case FelisCatusDembiaSketch::FixedPerch:
                                modifier += "FixedPerchTail"; break;
                            case FelisCatusDembiaSketch::FreePerch:
                                modifier += "FreePerchTail"; break;
                        }

                        // Only prepend underscore if this is not the zero-DOF
                        // model.
                        if (modifier != "") modifier = "_" + modifier;

                        FelisCatusDembiaSketch("Leland" + modifier,
                                "feliscatus" + modifier + ".osim",
                                canTwist[it], canHunch[ih], canWag[iw],
                                whichLegs[iL], whichTail[iT]);
                    }
                }
            }
        }
    }

    return EXIT_SUCCESS;
};

FelisCatusDembiaSketch::FelisCatusDembiaSketch(string modelName, string fileName,
        bool canTwist, bool canHunch, bool canWag,
        LegsType whichLegs, TailType whichTail)
{
	makeModel(modelName, fileName);

    createBaseCase();

    if (canTwist) {
		addTwist();
	}

	if (canHunch) {
		addHunch();
	}

	if (canWag) {
		addWag();
	}

	if (whichLegs == Rigid) {
		addRigidLegs();
    } else if (whichLegs == RetractBack) {
        addRetractLegs(false);
	} else if (whichLegs == Retract) {
		addRetractLegs();
	}

    if (whichTail != NoTail) {
        addTail(whichTail);
    }

	cat.print(fileName);
}

void FelisCatusDembiaSketch::createBaseCase()
{
    Body & ground = cat.getGroundBody();

    // Connecting the anterior body to the ground via a custom joint. 
	// Rotation is defined via YZX Euler angles, named yaw, pitch, and
	// roll respectively. The important translation is in Y, the direction
	// of gravity.	
	Vec3 locGAInGround(0);
    Vec3 orientGAInGround(0);
    Vec3 locGAInAnterior(0);
    Vec3 orientGAInAnterior(0);

	SpatialTransform groundAnteriorST;
	groundAnteriorST.updTransformAxis(0).setCoordinateNames(
            Array<string>("yaw", 1));
    groundAnteriorST.updTransformAxis(0).setAxis(Vec3(0, 1, 0));
    groundAnteriorST.updTransformAxis(1).setCoordinateNames(
            Array<string>("pitch", 1));
    groundAnteriorST.updTransformAxis(1).setAxis(Vec3(0, 0, 1));
    groundAnteriorST.updTransformAxis(2).setCoordinateNames(
            Array<string>("roll", 1));
    groundAnteriorST.updTransformAxis(2).setAxis(Vec3(1, 0, 0));
    groundAnteriorST.updTransformAxis(3).setCoordinateNames(
            Array<string>("tx", 1));
    groundAnteriorST.updTransformAxis(3).setAxis(Vec3(1, 0, 0));
    groundAnteriorST.updTransformAxis(4).setCoordinateNames(
            Array<string>("ty", 1));
    groundAnteriorST.updTransformAxis(4).setAxis(Vec3(0, 1, 0));
    groundAnteriorST.updTransformAxis(5).setCoordinateNames(
            Array<string>("tz", 1));
    groundAnteriorST.updTransformAxis(5).setAxis(Vec3(0, 0, 1));

    CustomJoint * groundAnterior = new CustomJoint("ground_anterior",
            ground, locGAInGround, orientGAInGround,
            *anteriorBody, locGAInAnterior, orientGAInAnterior,
            groundAnteriorST);

    CoordinateSet & groundAnteriorCS = groundAnterior->upd_CoordinateSet();
    // yaw
    double groundAnteriorCS0range[2] = {-Pi, Pi};
    groundAnteriorCS[0].setRange(groundAnteriorCS0range);
    groundAnteriorCS[0].setDefaultValue(0);
    groundAnteriorCS[0].setDefaultLocked(false);
    // pitch
    double groundAnteriorCS1range[2] = {-Pi, Pi};
    groundAnteriorCS[1].setRange(groundAnteriorCS1range);
    groundAnteriorCS[1].setDefaultValue(0);
    groundAnteriorCS[1].setDefaultLocked(false);
    // roll
    double groundAnteriorCS2range[2] = {-Pi, Pi};
    groundAnteriorCS[2].setRange(groundAnteriorCS2range);
    groundAnteriorCS[2].setDefaultValue(0);
    groundAnteriorCS[2].setDefaultLocked(false);
    // tx
    double groundAnteriorCS3range[2] = {-1, 1};
    groundAnteriorCS[3].setRange(groundAnteriorCS3range);
    groundAnteriorCS[3].setDefaultValue(0);
	groundAnteriorCS[3].setDefaultLocked(false);
    // ty
    double groundAnteriorCS4range[2] = {-1, 5};
    groundAnteriorCS[4].setRange(groundAnteriorCS4range);
    groundAnteriorCS[4].setDefaultValue(2);
	groundAnteriorCS[4].setDefaultLocked(false);
    // tz
    double groundAnteriorCS5range[2] = {-1, 1};
    groundAnteriorCS[5].setRange(groundAnteriorCS5range);
    groundAnteriorCS[5].setDefaultValue(0);
    groundAnteriorCS[5].setDefaultLocked(false);

    // Connecting the anterior and posterior bodies via a custom joint. 
	// Rotation is defined via ZYX Euler angles, named hunch, wag, and
	// twist respectively.
    Vec3 locAPInAnterior(0);
    Vec3 orientAPInAnterior(0);
    Vec3 locAPInPosterior(0);
    Vec3 orientAPInPosterior(0);

	SpatialTransform anteriorPosteriorST;
	anteriorPosteriorST.updTransformAxis(0).setCoordinateNames(
            Array<string>("hunch", 1));
    anteriorPosteriorST.updTransformAxis(0).setAxis(Vec3(0, 0, 1));
    anteriorPosteriorST.updTransformAxis(1).setCoordinateNames(
            Array<string>("wag", 1));
    anteriorPosteriorST.updTransformAxis(1).setAxis(Vec3(0, 1, 0));
    anteriorPosteriorST.updTransformAxis(2).setCoordinateNames(
            Array<string>("twist", 1));
	anteriorPosteriorST.updTransformAxis(2).setAxis(Vec3(1, 0, 0));

    CustomJoint * anteriorPosterior = new CustomJoint("anterior_posterior",
            *anteriorBody, locAPInAnterior, orientAPInAnterior,
            *posteriorBody, locAPInPosterior, orientAPInPosterior,
			anteriorPosteriorST);

    // Set coordinate limits based on empirical data (i.e., photos & video).
	CoordinateSet & anteriorPosteriorCS = anteriorPosterior->upd_CoordinateSet();
    // hunch
    double anteriorPosteriorCS0range[2] = {convertDegreesToRadians(-20),
										   convertDegreesToRadians(90)};  // -20/+90 deg
    anteriorPosteriorCS[0].setRange(anteriorPosteriorCS0range);
    anteriorPosteriorCS[0].setDefaultValue(0);
    anteriorPosteriorCS[0].setDefaultLocked(true);
	// wag
    double anteriorPosteriorCS1range[2] = {-0.25 * Pi, 0.25 * Pi};   // +/- 45 deg
    anteriorPosteriorCS[1].setRange(anteriorPosteriorCS1range);
    anteriorPosteriorCS[1].setDefaultValue(0);
    anteriorPosteriorCS[1].setDefaultLocked(true);
	// twist
    double anteriorPosteriorCS2range[2] = {convertDegreesToRadians(-80),
										   convertDegreesToRadians(80)};  // +/- 80 deg
    anteriorPosteriorCS[2].setRange(anteriorPosteriorCS2range);
    anteriorPosteriorCS[2].setDefaultValue(0);
    anteriorPosteriorCS[2].setDefaultLocked(true);

    cat.addBody(anteriorBody);
    cat.addBody(posteriorBody);
}

void FelisCatusDembiaSketch::addTwist()
{
	// Unlock twist.
	cat.updCoordinateSet().get("twist").setDefaultLocked(false);
	
	// Add twist actuator.
    addCoordinateActuator("twist");

	// Set twist limit force.
	CoordinateLimitForce * twistLimitForce = 
		new CoordinateLimitForce("twist", 80, 1.0E2, -80, 1.0E2, 1.0E1, 5.0, false);
	cat.addForce(twistLimitForce);
}

void FelisCatusDembiaSketch::addHunch()
{
	// Unlock hunch and change default so that cat is initially hunched.
	cat.updCoordinateSet().get("hunch").setDefaultLocked(false);
	cat.updCoordinateSet().get("hunch").setDefaultValue(convertDegreesToRadians(30));
	cat.updCoordinateSet().get("pitch").setDefaultValue(convertDegreesToRadians(-15));
	
	// Add hunch actuator.
    addCoordinateActuator("hunch");

	// Set hunch limit force
	CoordinateLimitForce * hunchLimitForce = 
		new CoordinateLimitForce("hunch", 90, 1.0E2, -20, 1.0E2, 1.0E1, 2.0, false);
	cat.addForce(hunchLimitForce);
}

void FelisCatusDembiaSketch::addWag()
{
	// Unlock wag.
	cat.updCoordinateSet().get("wag").setDefaultLocked(false);
	
	// Add wag actuator.
    addCoordinateActuator("wag");

	// Set wag limit force.
	CoordinateLimitForce * wagLimitForce = 
		new CoordinateLimitForce("wag", 45, 1.0E2, -45, 1.0E2, 1.0E1, 2.0, false);
	cat.addForce(wagLimitForce);
}

void FelisCatusDembiaSketch::addRigidLegs()
{
	Vec3 locALegsInAnterior(-0.75 * segmentalLength, 0.5 * segmentalDiam, 0);
    Vec3 orientALegsInAnterior(0);
    Vec3 locALegsInLegs(0);
	State& state = cat.initSystem();
	double pitch = cat.getCoordinateSet().get("pitch").getValue(state);
    Vec3 orientALegsInLegs(0, 0, -0.5 * Pi + pitch);
    WeldJoint * anteriorToLegs = new WeldJoint("anterior_legs",
            *anteriorBody, locALegsInAnterior, orientALegsInAnterior,
            *anteriorLegs, locALegsInLegs, orientALegsInLegs);

	Vec3 locPLegsInPosterior(0.75 * segmentalLength, 0.5 * segmentalDiam, 0);
    Vec3 orientPLegsInPosterior(0, Pi, 0);
    Vec3 locPLegsInLegs(0);
	
    Vec3 orientPLegsInLegs(0, 0, -0.5 * Pi + pitch);
    WeldJoint * posteriorToLegs = new WeldJoint("posterior_legs",
            *posteriorBody, locPLegsInPosterior, orientPLegsInPosterior,
            *posteriorLegs, locPLegsInLegs, orientPLegsInLegs);

	cat.addBody(anteriorLegs);
	cat.addBody(posteriorLegs);
}

void FelisCatusDembiaSketch::addRetractLegs(bool frontLegsRetract)
{
	// Adding 1-DOF legs.
	Vec3 locALegsInAnterior(-0.75 * segmentalLength, 0.5 * segmentalDiam, 0);
    Vec3 orientALegsInAnterior(0);
    Vec3 locALegsInLegs(0);
    Vec3 orientALegsInLegs(0, 0, -0.5 * Pi);
    PinJoint * anteriorToLegs = new PinJoint("anterior_legs",
            *anteriorBody, locALegsInAnterior, orientALegsInAnterior,
            *anteriorLegs, locALegsInLegs, orientALegsInLegs);
    CoordinateSet & anteriorToLegsCS = anteriorToLegs->upd_CoordinateSet();
    anteriorToLegsCS[0].setName("frontLegs");
    double anteriorToLegsCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    anteriorToLegsCS[0].setRange(anteriorToLegsCS0range);
    State& state = cat.initSystem();
	double pitch = cat.getCoordinateSet().get("pitch").getValue(state);
	if (frontLegsRetract)
    {
        anteriorToLegsCS[0].setDefaultValue(-pitch);
        anteriorToLegsCS[0].setDefaultLocked(false);
    }
    else
    {
        anteriorToLegsCS[0].setDefaultValue(0.25 * Pi);
        anteriorToLegsCS[0].setDefaultLocked(true);
    }

	Vec3 locPLegsInPosterior(0.75 * segmentalLength, 0.5 * segmentalDiam, 0);
    Vec3 orientPLegsInPosterior(0, Pi, 0);
    Vec3 locPLegsInLegs(0);
    Vec3 orientPLegsInLegs(0, 0, -0.5 * Pi);
    PinJoint * posteriorToLegs = new PinJoint("posterior_legs",
            *posteriorBody, locPLegsInPosterior, orientPLegsInPosterior,
            *posteriorLegs, locPLegsInLegs, orientPLegsInLegs);
    CoordinateSet & posteriorToLegsCS = posteriorToLegs->upd_CoordinateSet();
    posteriorToLegsCS[0].setName("backLegs");
    double posteriorToLegsCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    posteriorToLegsCS[0].setRange(posteriorToLegsCS0range);
    posteriorToLegsCS[0].setDefaultValue(-pitch);
    posteriorToLegsCS[0].setDefaultLocked(false);

	cat.addBody(anteriorLegs);
	cat.addBody(posteriorLegs);

	// Adding leg actuators and limit forces.
    if (frontLegsRetract)
    {
        addCoordinateActuator("frontLegs");

		CoordinateLimitForce * frontLegsLimitForce = 
			new CoordinateLimitForce("frontLegs", 90, 1.0E2, -90, 1.0E2, 1.0E1, 2.0, false);
		cat.addForce(frontLegsLimitForce);
    }
    
    addCoordinateActuator("backLegs");

	CoordinateLimitForce * backLegsLimitForce = 
		new CoordinateLimitForce("backLegs", 90, 1.0E2, -90, 1.0E2, 1.0E1, 2.0, false);
	cat.addForce(backLegsLimitForce);
}

void FelisCatusDembiaSketch::addTail(TailType whichTail)
{
    // For now, only considering Two-DOF tail: one that can 'pitch', then can
    // rotate in a cone with that pitch.

    // Connecting posterior and tail bodies via a custom joint.
	// Rotation is defined via XZ Euler angles:
    // shake is rotation of the tail about a cone whose half-angle (?) is
    // perch.
    double tailMassFactor = 0.5;
    double tailSizeFactor = 1.2;

    Body * tailBody = new Body();
    tailBody->setName("tailBody");
    tailBody->setMass(tailMassFactor * segmentalMass);
    tailBody->setMassCenter(Vec3(tailSizeFactor * segmentalLength, 0, 0));
    tailBody->setInertia(zeroInertia);

    Vec3 locPTInPosterior(segmentalLength, 0, 0);
    Vec3 orientPTInPosterior(0);
    Vec3 locPTInTail(0);
    Vec3 orientPTInTail(0);

	SpatialTransform posteriorTailST;
	posteriorTailST.updTransformAxis(0).setCoordinateNames(
            Array<string>("shake", 1));
    posteriorTailST.updTransformAxis(0).setAxis(Vec3(1, 0, 0));
    posteriorTailST.updTransformAxis(1).setCoordinateNames(
            Array<string>("perch", 1));
    posteriorTailST.updTransformAxis(1).setAxis(Vec3(0, 0, 1));
    // Unused, but was causing 'colinear axies' error (default was 0 0 1).
    posteriorTailST.updTransformAxis(2).setAxis(Vec3(0, 1, 0));

    CustomJoint * posteriorTail = new CustomJoint("posterior_tail",
            *posteriorBody, locPTInPosterior, orientPTInPosterior,
            *tailBody, locPTInTail, orientPTInTail,
			posteriorTailST);

    CoordinateSet & posteriorTailCS = posteriorTail->upd_CoordinateSet();
    // shake
    double posteriorTailCS0range[2] = {-2.0 * Pi, 2.0 * Pi};
    posteriorTailCS[0].setRange(posteriorTailCS0range);
    posteriorTailCS[0].setDefaultValue(0);
    posteriorTailCS[0].setDefaultLocked(false);
    // perch
    double posteriorTailCS1range[2] = {0, 0.5 * Pi};
    posteriorTailCS[1].setRange(posteriorTailCS1range);
    posteriorTailCS[1].setDefaultValue(0.25 * Pi);
    if (whichTail == FixedPerch)
        posteriorTailCS[1].setDefaultLocked(true);
    else if (whichTail == FreePerch)
        posteriorTailCS[1].setDefaultLocked(false);

    // Display.
    DisplayGeometry * tailBodyDisplay = new DisplayGeometry("sphere.vtp");
    tailBodyDisplay->setOpacity(0.5);
    tailBodyDisplay->setTransform(Transform(Vec3(0.5 * tailSizeFactor * segmentalLength, 0, 0)));
    tailBodyDisplay->setScaleFactors(
            Vec3(tailSizeFactor * segmentalLength,
                0.1 * tailSizeFactor * segmentalLength,
                0.1 * tailSizeFactor * segmentalLength));
    tailBody->updDisplayer()->updGeometrySet().adoptAndAppend(tailBodyDisplay);
    tailBody->updDisplayer()->setShowAxes(true);

    cat.addBody(tailBody);

    // Actuation.
    addCoordinateActuator("shake");

    if (whichTail == FreePerch)
        addCoordinateActuator("perch");
}

void FelisCatusDembiaSketch::addCoordinateActuator(string coordinateName)
{
    CoordinateActuator * act = new CoordinateActuator(coordinateName);
    act->setName(coordinateName + "_actuator");
    act->setMinControl(-_maxTorque);
    act->setMaxControl(_maxTorque);
    cat.addForce(act);
}
