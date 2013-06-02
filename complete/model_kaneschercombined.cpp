
#include "FelisCatusModeling.h"

using OpenSim::FreeJoint;

/**
 * Creates an OpenSim model of a cat, sufficient for exploring the cat's
 * righting reflex.
 *
 * This model allows for the "salient features of motion" of a falling cat
 * first identified by Kane and Scher (1969). These include:
 * 	1) The torso bends but does not twist.
 * 	2) At release, the spine is bent forward. Afterwards, the spine is
 * 	   first bent to one side, the backward, then to the other side,
 * 	   and finally forwards again. At the end of this "oscillation,"
 * 	   the cat has turned over.
 * 	3) The backwards bend that the cat experiences during its turning
 * 	   maneuver is more pronounced that its initial or final forward
 * 	   bends.
 * We combine figures 2 and 3 from the paper by trying to introduce a loop in
 * the kinematic graph.
 *
 * NOTE The model is not valid, and thus this code is incomplete.
 *
 * We use the general convention that body origins are at joint locations,
 * and thus mass centers are specified to be some distance away. Furthermore,
 * the model contains several "dummy" frames (i.e., massless
 * bodies) that are used to specify rotational relations between the anterior
 * and posterior segments of the cat.
 * */
class KaneScherCombinedModeling : public FelisCatusModeling
{
    void addJoints();
};

/** Creates a FelisCatus model with a spinal joint defined by Kane and Scher,
 * with the topology based off Figure 3 of Kane and Scher (1969).
 * */
int main(int argc, char *argv[])
{
    KaneScherCombinedModeling m;
    m.makeModel("Leland_kaneschercombined",
            "feliscatus_kaneschercombined.osim");

    return EXIT_SUCCESS;
}

/**
 * The topology with this joint is:
 * 
 *                   Aintermed -> anteriorBody -> K -> P -> B2
 *                 /                                        |
 * ground -> frameQ                                         |
 *                 \                                        |
 *                   Bintermed -> posteriorBody <------------
 * 
 * */
void KaneScherCombinedModeling::addJoints()
{
    Body & ground = cat.getGroundBody();

    // I'm calling massless bodies "frames". Here are all the ones I'm
    // using:
    // The frame containing A1 and B1.
    Body * frameQ = newMasslessBody("Q");
    // Frame rotated from Q about N by -gamma/2, to which anteriorBody is
    // attached.
    Body * Aintermed = newMasslessBody("Aintermed");
    // Frame rotated from Q about N by gamma/2, to which posteriorBody is
    // attached.
    Body * Bintermed = newMasslessBody("Bintermed");

    // -- Connect Q to ground.
    Vec3 locGQInGround(0);
    // TODO need some description here.
    // Y rotation to place the joint axis in the ground's X-Y plane.
    Vec3 orientGQInGround(0, 0.5 * Pi, 0);
    // TODO X rotation to offset the fixed location of the joint axis in
    // space,
    // TODO so that it is inclined from the ground X axis, toward the
    // ground Y axis.
    // TODO Vec3 orientGQInGround(0.25 * Pi, 0.5 * Pi, 0);
    Vec3 locGQInFrameQ(0);
    // Y rotation to undo previous rotation on other side of joint so that
    // cat lies in X-Y plane.
    // Z rotation to set the default configuration of the cat to be upside
    // down.
    Vec3 orientGQInFrameQ(0, -0.5 * Pi, Pi);
    PinJoint * groundFrameQ = new PinJoint("ground_frameQ",
            ground, locGQInGround, orientGQInGround,
            *frameQ, locGQInFrameQ, orientGQInFrameQ);
    CoordinateSet & groundFrameQCS = groundFrameQ->upd_CoordinateSet();
    groundFrameQCS[0].setName("kane_psi");
    double groundFrameQCS0range[2] = {-1.5 * Pi, 1.5 * Pi};
    groundFrameQCS[0].setRange(groundFrameQCS0range);
    groundFrameQCS[0].setDefaultValue(0.0);
    groundFrameQCS[0].setDefaultLocked(false);
    // TODO Change the above to a CustomJoint.

    // Connect Aintermed(iate) to Q.
    Vec3 locQAIinFrameQ(0);
    Vec3 orientQAIinFrameQ(0, 0, 0);
    Vec3 locQAIinAintermed(0);
    Vec3 orientQAIinAintermed(0, 0, 0);
    PinJoint * frameQAintermed = new PinJoint("frameQ_Aintermed",
            *frameQ, locQAIinFrameQ, orientQAIinFrameQ,
            *Aintermed, locQAIinAintermed, orientQAIinAintermed);
    CoordinateSet & frameQAintermedCS =
        frameQAintermed->upd_CoordinateSet();
    frameQAintermedCS[0].setName("kane_gammaA");
    double frameQAintermedCS0range[2] = {-0.5 * Pi, 0.25 * Pi};
    frameQAintermedCS[0].setRange(frameQAintermedCS0range);
    frameQAintermedCS[0].setDefaultValue(0.0);
    frameQAintermedCS[0].setDefaultLocked(false);

    // Connect Bintermed(iate) to Q.
    Vec3 locQBIinFrameQ(0);
    Vec3 orientQBIinFrameQ(0, 0, 0);
    Vec3 locQBIinBintermed(0);
    Vec3 orientQBIinBintermed(0, 0, 0);
    PinJoint * frameQBintermed = new PinJoint("frameQ_Bintermed",
            *frameQ, locQBIinFrameQ, orientQBIinFrameQ,
            *Bintermed, locQBIinBintermed, orientQBIinBintermed);
    CoordinateSet & frameQBintermedCS =
        frameQBintermed->upd_CoordinateSet();
    frameQBintermedCS[0].setName("kane_gammaB");
    double frameQBintermedCS0range[2] = {0, 0.5 * Pi};
    frameQBintermedCS[0].setRange(frameQBintermedCS0range);
    frameQBintermedCS[0].setDefaultValue(0.0);
    frameQBintermedCS[0].setDefaultLocked(false);

    // Connect Aintermed to anteriorBody.
    Vec3 locAIAinAintermed(0);
    Vec3 orientAIAinAintermed(0, 0.5 * Pi, 0);
    Vec3 locAIAinAnterior(0);
    Vec3 orientAIAinAnterior(0, -0.5 * Pi, 0);
    PinJoint * AintermedAnterior = new PinJoint("Aintermed_anterior",
            *Aintermed, locAIAinAintermed, orientAIAinAintermed,
            *anteriorBody, locAIAinAnterior, orientAIAinAnterior);
    CoordinateSet & AintermedAnteriorCS =
        AintermedAnterior->upd_CoordinateSet();
    // "integ" because this coord is the integral of the speed u.
    AintermedAnteriorCS[0].setName("kane_u_integ");
    double AintermedAnteriorCS0range[2] = {-2 * Pi, 2 * Pi};
    AintermedAnteriorCS[0].setRange(AintermedAnteriorCS0range);
    AintermedAnteriorCS[0].setDefaultValue(0.0);
    AintermedAnteriorCS[0].setDefaultLocked(false);

    // Connect Bintermed to posteriorBody.
    Vec3 locBIBinBintermed(0);
    Vec3 orientBIBinBintermed(0, 0.5 * Pi, 0);
    Vec3 locBIBinPosterior(0);
    Vec3 orientBIBinPosterior(0, -0.5 * Pi, 0);
    PinJoint * BintermedPosterior = new PinJoint("Bintermed_posterior",
            *Bintermed, locBIBinBintermed, orientBIBinBintermed,
            *posteriorBody, locBIBinPosterior, orientBIBinPosterior);
    CoordinateSet & BintermedPosteriorCS =
        BintermedPosterior->upd_CoordinateSet();
    BintermedPosteriorCS[0].setName("kane_v_integ");
    double BintermedPosteriorCS0range[2] = {-2 * Pi, 2 * Pi};
    BintermedPosteriorCS[0].setRange(BintermedPosteriorCS0range);
    BintermedPosteriorCS[0].setDefaultValue(0.0);
    BintermedPosteriorCS[0].setDefaultLocked(false);

    // ------------------------------------------------------------------------
    // --- Sean's Figure 2 code.
    //
    // Frame in which the X-axis points along ray K. K lies in the A1-A2
	// plane, so frame K is formed by rotation (by angle alpha) about A3.
    Body* frameK = newMasslessBody("K");
	Vec3 locationAKInAnterior(0);
	Vec3 orientationAKInAnterior(0);
	Vec3 locationAKinFrameK(0);
	Vec3 orientationAKinFrameK(0);
	PinJoint* anteriorFrameK = new PinJoint("anterior_K",
		*anteriorBody, locationAKInAnterior, orientationAKInAnterior,
		*frameK, locationAKinFrameK, orientationAKinFrameK, false);
	CoordinateSet& anteriorFrameKCS = anteriorFrameK->upd_CoordinateSet();
	anteriorFrameKCS[0].setName("kane_alpha");
	//double anteriorFrameKCS0range[2] = {convertDegreesToRadians(35),
	//	convertDegreesToRadians(85)};
	double anteriorFrameKCS0range[2] = {-Pi, 0};
	anteriorFrameKCS[0].setRange(anteriorFrameKCS0range);
	//anteriorFrameKCS[0].setDefaultValue(convertDegreesToRadians(60));
	anteriorFrameKCS[0].setDefaultValue(0);
	anteriorFrameKCS[0].setDefaultLocked(true);

    // Defining the following axes:
    //		B1: X-axis of coordinate frame attached to the posterior body
    //		B2: normal to B1 and lying in the plane defined by B1 and ray K
    //		B3: mutually perpendicular to B1 and B2

	// Frame with Z-axis as B3 (named P in Kane and Scher), formed by rotation
	// about K by angle theta.
    Body* frameP = newMasslessBody("P");
	Vec3 locationKPInFrameK(0);
	Vec3 orientationKPInFrameK(0, 0.5 * Pi, 0);
	Vec3 locationKPInFrameP(0);
	Vec3 orientationKPInFrameP(0, -0.5 * Pi, 0);
	PinJoint* FrameKFrameP = new PinJoint("K_P",
		*frameK, locationKPInFrameK, orientationKPInFrameK,
		*frameP, locationKPInFrameP, orientationKPInFrameP, false);
	CoordinateSet& FrameKFramePCS = FrameKFrameP->upd_CoordinateSet();
	FrameKFramePCS[0].setName("kane_theta");
	double FrameKFramePCS0range[2] = {0, 2 * Pi};
	FrameKFramePCS[0].setRange(FrameKFramePCS0range);
	FrameKFramePCS[0].setDefaultValue(0);
	FrameKFramePCS[0].setDefaultLocked(false);

	// Frame with Y-axis as B2, formed by rotation about B3 by angle beta.
	Body* frameB2 = newMasslessBody("B2");
	Vec3 locationPB2InFrameB3(0);
	Vec3 orientationPB2InFrameB3(0);
	Vec3 locationPB2InFrameB2(0);
	Vec3 orientationPB2InFrameB2(0);
	PinJoint* FramePFrameB2 = new PinJoint("P_B2",
		*frameP, locationPB2InFrameB3, orientationPB2InFrameB3,
		*frameB2, locationPB2InFrameB2, orientationPB2InFrameB2, false);
	CoordinateSet& FramePFrameB2CS = FramePFrameB2->upd_CoordinateSet();
	FramePFrameB2CS[0].setName("kane_beta");
	//double FramePFrameB2CS0range[2] = {convertDegreesToRadians(60),
	//	convertDegreesToRadians(90)};
	double FramePFrameB2CS0range[2] = {0, Pi};
	FramePFrameB2CS[0].setRange(FramePFrameB2CS0range);
	//FramePFrameB2CS[0].setDefaultValue(convertDegreesToRadians(75));
	FramePFrameB2CS[0].setDefaultValue(0);
	FramePFrameB2CS[0].setDefaultLocked(true);

	// Connecting the posterior body (i.e., B1).
    /* TODO what would ideally be here.
	Vec3 locationB2PostInFrameB2(0);
	Vec3 orientationB2PostInFrameB2(0, 0, -0.5 * Pi);
	Vec3 locationB2PostInPosterior(0);
	Vec3 orientationB2PostInPosterior(Pi, 0, 0.5 * Pi);
	PinJoint* FrameB2Posterior = new PinJoint("B2_posterior",
		*frameB2, locationB2PostInFrameB2, orientationB2PostInFrameB2,
        *posteriorBody, locationB2PostInPosterior,
        orientationB2PostInPosterior, false);
	CoordinateSet& FrameB2PosteriorCS = FrameB2Posterior->upd_CoordinateSet();
	FrameB2PosteriorCS[0].setName("kane_s");
	double FrameB2PosteriorCS0range[2] = {-Pi, Pi};
	FrameB2PosteriorCS[0].setRange(FrameB2PosteriorCS0range);
	FrameB2PosteriorCS[0].setDefaultValue(0);
	FrameB2PosteriorCS[0].setDefaultLocked(false);
    */
	Vec3 locationB2PostInFrameB2(0);
	Vec3 orientationB2PostInFrameB2(0, 0, -0.5 * Pi);
	Vec3 locationB2PostInPosterior(0);
	Vec3 orientationB2PostInPosterior(Pi, 0, 0.5 * Pi);
	FreeJoint* FrameB2Posterior = new FreeJoint("B2_posterior",
		*frameB2, locationB2PostInFrameB2, orientationB2PostInFrameB2,
        *posteriorBody, locationB2PostInPosterior,
        orientationB2PostInPosterior, false);
	CoordinateSet& FrameB2PosteriorCS = FrameB2Posterior->upd_CoordinateSet();
	FrameB2PosteriorCS[0].setName("temp_free0");
	double FrameB2PosteriorCS0range[2] = {-Pi, Pi};
	FrameB2PosteriorCS[0].setRange(FrameB2PosteriorCS0range);
	FrameB2PosteriorCS[0].setDefaultValue(0);
	FrameB2PosteriorCS[0].setDefaultLocked(false);

	FrameB2PosteriorCS[1].setName("temp_free1");
	FrameB2PosteriorCS[1].setRange(FrameB2PosteriorCS0range);
	FrameB2PosteriorCS[1].setDefaultValue(0);
	FrameB2PosteriorCS[1].setDefaultLocked(false);

	FrameB2PosteriorCS[2].setName("temp_free2");
	FrameB2PosteriorCS[2].setRange(FrameB2PosteriorCS0range);
	FrameB2PosteriorCS[2].setDefaultValue(0);
	FrameB2PosteriorCS[2].setDefaultLocked(false);

	FrameB2PosteriorCS[3].setName("temp_free3");
	FrameB2PosteriorCS[3].setRange(FrameB2PosteriorCS0range);
	FrameB2PosteriorCS[3].setDefaultValue(0);
	FrameB2PosteriorCS[3].setDefaultLocked(false);

	FrameB2PosteriorCS[4].setName("temp_free4");
	FrameB2PosteriorCS[4].setRange(FrameB2PosteriorCS0range);
	FrameB2PosteriorCS[4].setDefaultValue(0);
	FrameB2PosteriorCS[4].setDefaultLocked(false);

	FrameB2PosteriorCS[5].setName("temp_free5");
	FrameB2PosteriorCS[5].setRange(FrameB2PosteriorCS0range);
	FrameB2PosteriorCS[5].setDefaultValue(0);
	FrameB2PosteriorCS[5].setDefaultLocked(false);
    // ------------------------------------------------------------------------

    // --- Display geometry for the time being to help see relation to Fig 3.
    // This geometry represents the plane/frame Q.
    DisplayGeometry * frameQDisplay = new DisplayGeometry("box.vtp");
    frameQDisplay->setOpacity(0.2);
    frameQDisplay->setScaleFactors(Vec3(2, 2, 0.01));

    frameQ->updDisplayer()->updGeometrySet().adoptAndAppend(frameQDisplay);
    frameQ->updDisplayer()->setShowAxes(true);

    // --- Add bodies.
    // Need to do this before the constraints.
    cat.addBody(frameQ);
    cat.addBody(Aintermed);
    cat.addBody(Bintermed);
    cat.addBody(anteriorBody);
    cat.addBody(posteriorBody);
    // Sean's figure 2 bodies.
    cat.addBody(frameK);
    cat.addBody(frameP);
    cat.addBody(frameB2);

    // --- Constraints.
    // -- Constraint between intermediate frames.
    CoordinateCouplerConstraint * intermedConstr = 
        new CoordinateCouplerConstraint();
    intermedConstr->setName("intermed");
    intermedConstr->setIndependentCoordinateNames(
            Array<string>("kane_gammaA", 1));
    intermedConstr->setDependentCoordinateName("kane_gammaB");
    Array<double> intermedConstrFcnCoeff;
    // TODO I don't understand hw these coefficients work and
    // I think the documentation is insufficient.
    intermedConstrFcnCoeff.append(-1);
    intermedConstr->setFunction(new LinearFunction(intermedConstrFcnCoeff));

    // -- Constraint between kane_u_integ and kane_v_integ.
    CoordinateCouplerConstraint * twistConstr = 
        new CoordinateCouplerConstraint();
    twistConstr->setName("twist");
    twistConstr->setIndependentCoordinateNames(
            Array<string>("kane_u_integ", 1));
    twistConstr->setDependentCoordinateName("kane_v_integ");
    Array<double> twistConstrFcnCoeff;
    // TODO I don't understand how these coefficients work and
    // I think the documentation is insufficient.
    twistConstrFcnCoeff.append(1);
    twistConstr->setFunction(new LinearFunction(twistConstrFcnCoeff));

    // -- Add constraints to model.
    cat.addConstraint(intermedConstr);
    cat.addConstraint(twistConstr);

}
