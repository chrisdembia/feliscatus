
#include "FelisCatusModeling.h"

/**
 * This models the cat as two elliptical segments connected by a
 * one degree-of-freedom (back flexion) joint. It also includes
 * "rigid" legs that offset the cat's center of mass.
 * */

class FlexRigidLegsModeling : public FelisCatusModeling
{
    void addJoints();
    void addActuators();
};

int main(int argc, char *argv[])
{
    FlexRigidLegsModeling m = FlexRigidLegsModeling();
    m.makeModel("Leland_flexRigidLegs",
            "feliscatus_flexRigidLegs.osim");

    return EXIT_SUCCESS;
};

void FlexRigidLegsModeling::addJoints()
{
    Body & ground = cat.getGroundBody();

    // Connecting the anterior body to the ground via a custom joint. 
	// Rotation is defined via ZXY Euler angles, named pitch, roll, and
	// yaw respectively. The important translation is in Y, the direction
	// of gravity.	
	SpatialTransform groundAnteriorST;
    groundAnteriorST.updTransformAxis(0).setCoordinateNames(
            Array<string>("pitch", 1));
    groundAnteriorST.updTransformAxis(0).setAxis(Vec3(0, 0, 1));
    groundAnteriorST.updTransformAxis(1).setCoordinateNames(
            Array<string>("roll", 1));
    groundAnteriorST.updTransformAxis(1).setAxis(Vec3(1, 0, 0));
    groundAnteriorST.updTransformAxis(2).setCoordinateNames(
            Array<string>("yaw", 1));
    groundAnteriorST.updTransformAxis(2).setAxis(Vec3(0, 1, 0));
    groundAnteriorST.updTransformAxis(3).setCoordinateNames(
            Array<string>("tx", 1));
    groundAnteriorST.updTransformAxis(3).setAxis(Vec3(1, 0, 0));
    groundAnteriorST.updTransformAxis(4).setCoordinateNames(
            Array<string>("ty", 1));
    groundAnteriorST.updTransformAxis(4).setAxis(Vec3(0, 1, 0));
    groundAnteriorST.updTransformAxis(5).setCoordinateNames(
            Array<string>("tz", 1));
    groundAnteriorST.updTransformAxis(5).setAxis(Vec3(0, 0, 1));

    Vec3 locGAInGround(0);
    Vec3 orientGAInGround(0);
    Vec3 locGAInAnterior(0);
    Vec3 orientGAInAnterior(0);

    CustomJoint* groundAnterior = new CustomJoint("ground_anterior",
            ground, locGAInGround, orientGAInGround,
            *anteriorBody, locGAInAnterior, orientGAInAnterior,
            groundAnteriorST);

    CoordinateSet & groundAnteriorCS = groundAnterior->upd_CoordinateSet();
    // pitch
    double groundAnteriorCS0range[2] = {-Pi, Pi};
    groundAnteriorCS[0].setRange(groundAnteriorCS0range);
    groundAnteriorCS[0].setDefaultValue(0);
    groundAnteriorCS[0].setDefaultLocked(false);
    // roll
    double groundAnteriorCS1range[2] = {-Pi, Pi};
    groundAnteriorCS[1].setRange(groundAnteriorCS1range);
    groundAnteriorCS[1].setDefaultValue(0);
    groundAnteriorCS[1].setDefaultLocked(false);
    // yaw
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

    // Connecting the anterior and posterior bodies.
    Vec3 locAPInAnterior(-0.4 * segmentalLength, 0, 0);
    Vec3 orientAPInAnterior(0);
    Vec3 locAPInPosterior(0.4 * segmentalLength, 0, 0);
    Vec3 orientAPInPosterior(0);
    BallJoint * anteriorPosterior = new BallJoint("anterior_posterior",
            *anteriorBody, locAPInAnterior, orientAPInAnterior,
            *posteriorBody, locAPInPosterior, orientAPInPosterior);

    CoordinateSet & anteriorPosteriorCS =
        anteriorPosterior->upd_CoordinateSet();
    anteriorPosteriorCS[0].setName("twist");
    double anteriorPosteriorCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    anteriorPosteriorCS[0].setRange(anteriorPosteriorCS0range);
    anteriorPosteriorCS[0].setDefaultValue(0);
    anteriorPosteriorCS[0].setDefaultLocked(true);
	anteriorPosteriorCS[1].setName("jive");
    double anteriorPosteriorCS1range[2] = {-0.5 * Pi, 0.5 * Pi};
    anteriorPosteriorCS[1].setRange(anteriorPosteriorCS1range);
    anteriorPosteriorCS[1].setDefaultValue(0);
    anteriorPosteriorCS[1].setDefaultLocked(true);
	anteriorPosteriorCS[2].setName("hunch");
    double anteriorPosteriorCS2range[2] = {-0.5 * Pi, 0.5 * Pi};
    anteriorPosteriorCS[2].setRange(anteriorPosteriorCS2range);
    anteriorPosteriorCS[2].setDefaultValue(0);
    anteriorPosteriorCS[2].setDefaultLocked(false);

    // Adding legs to each half.
	Vec3 locALegsInAnterior(0.25 * segmentalLength, 0.25 * segmentalDiam, 0);
    Vec3 orientALegsInAnterior(0);
    Vec3 locALegsInLegs(0.5 * legsLength, 0, 0);
    Vec3 orientALegsInLegs(0, 0, 0.5 * Pi);
    WeldJoint * anteriorToLegs = new WeldJoint("anterior_legs",
            *anteriorBody, locALegsInAnterior, orientALegsInAnterior,
            *anteriorLegs, locALegsInLegs, orientALegsInLegs);

	Vec3 locPLegsInPosterior(-0.25 * segmentalLength, 0.25 * segmentalDiam, 0);
    Vec3 orientPLegsInPosterior(0);
    Vec3 locPLegsInLegs(0.5 * legsLength, 0, 0);
    Vec3 orientPLegsInLegs(0, 0, 0.5 * Pi);
    WeldJoint * posteriorToLegs = new WeldJoint("posterior_legs",
            *posteriorBody, locPLegsInPosterior, orientPLegsInPosterior,
            *posteriorLegs, locPLegsInLegs, orientPLegsInLegs);

    cat.addBody(anteriorBody);
    cat.addBody(posteriorBody);
	cat.addBody(anteriorLegs);
	cat.addBody(posteriorLegs);
}

void FlexRigidLegsModeling::addActuators()
{
    double maxTorque = 100; // N-m
    CoordinateActuator * hunchAct = new CoordinateActuator("hunch");

    hunchAct->setName("hunch_actuator");

    hunchAct->setMinControl(-maxTorque);
    hunchAct->setMaxControl(maxTorque);

    cat.addForce(hunchAct);
}
