
#include "FelisCatusModeling.h"

using OpenSim::CoordinateCouplerConstraint;
using OpenSim::CustomJoint;
using OpenSim::LinearFunction;
using OpenSim::PinJoint;
using OpenSim::PointToPointSpring;
using OpenSim::SpatialTransform;

using SimTK::convertDegreesToRadians;

/**
 * This model allows for the "salient features of motion" of a falling cat
 * first identified by Kane and Scher (1969). These include:
 * 
 * 	1) The torso bends but does not twist.
 * 	2) At release, the spine is bent forward. Afterwards, the spine is
 * 	   first bent to one side, then backward, then to the other side,
 * 	   and finally forwards again. At the end of this "oscillation,"
 * 	   the cat has turned over.
 * 	3) The backwards bend that the cat experiences during its turning
 * 	   maneuver is more pronounced that its initial or final forward
 * 	   bends.
 * 
 * These salient features are implemented starting with Figure 2 of their paper.
 *
 * NOTE: In its current state, this code does not correctly implement Kane and
 * Scher's model.
 * */
class KaneScherFig2Modeling : public FelisCatusModeling
{
    void addBaseJointsAndBodies();
	void addBaseForcesAndActuation();
};

/** 
 * Creates a FelisCatus model with a spinal joint defined by Kane and Scher,
 * with the topology based off Figure 2 of Kane and Scher (1969). Starting with
 * the anterior half of the cat, each body in the model is simply pinned to its
 * parent; this gives it one DOF relative to the parent. Although not the most
 * efficient way to build up the model (e.g., as opposed to using Euler angles),
 * it clearly shows the dependencies in the model and the relations between the
 * cat's anterior and posterior bodies.
 * */
int main(int argc, char *argv[])
{
    KaneScherFig2Modeling m;
    m.makeModel("Leland_kanescherfig2");
    m.printModel("feliscatus_kanescherfig2.osim");

    return EXIT_SUCCESS;
}

void KaneScherFig2Modeling::addBaseJointsAndBodies()
{	
	// NOTE: Massless bodies are simply coordinate frames for rotation. We
    // ----  have attempted to be as faithful to Kane and Scher (1969) as
    //       possible. See their paper to understand the notation used.

	// BODY CONNECTIONS:  ground -> anterior -> K -> P -> B2 -> posterior
	//
	// JOINTS/COORDINATES:  custom (u_integ) -> pin (alpha) -> pin (theta) ->
	//						pin (beta) -> pin (v_integ)

	// Connecting the anterior body (A) to the ground via a custom joint (same
	// as in 'model_dembiasketch'). Rotation is defined via ZXY Euler angles,
	// named flip, u_integ (AKA. sprawl), and roll respectively. The important
	// translation is in Y, the direction of gravity.
	Body& ground = _cat.getGroundBody();
	
	SpatialTransform groundAnteriorST;
    groundAnteriorST.updTransformAxis(0).setCoordinateNames(
            Array<string>("anterior_flip", 1));
    groundAnteriorST.updTransformAxis(0).setAxis(Vec3(0, 0, 1));
    groundAnteriorST.updTransformAxis(1).setCoordinateNames(
            Array<string>("kane_u_integ", 1));
    groundAnteriorST.updTransformAxis(1).setAxis(Vec3(1, 0, 0));
    groundAnteriorST.updTransformAxis(2).setCoordinateNames(
            Array<string>("anterior_roll", 1));
    groundAnteriorST.updTransformAxis(2).setAxis(Vec3(0, 1, 0));
    groundAnteriorST.updTransformAxis(3).setCoordinateNames(
            Array<string>("anterior_tx", 1));
    groundAnteriorST.updTransformAxis(3).setAxis(Vec3(1, 0, 0));
    groundAnteriorST.updTransformAxis(4).setCoordinateNames(
            Array<string>("anterior_ty", 1));
    groundAnteriorST.updTransformAxis(4).setAxis(Vec3(0, 1, 0));
    groundAnteriorST.updTransformAxis(5).setCoordinateNames(
            Array<string>("anterior_tz", 1));
    groundAnteriorST.updTransformAxis(5).setAxis(Vec3(0, 0, 1));

    Vec3 locGAInGround(0);
    Vec3 orientGAInGround(0);
    Vec3 locGAInAnterior(0);
    Vec3 orientGAInAnterior(Pi, 0, 0);

    CustomJoint* groundAnterior = new CustomJoint("ground_anterior",
            ground, locGAInGround, orientGAInGround,
            *_anteriorBody, locGAInAnterior, orientGAInAnterior,
            groundAnteriorST);

    CoordinateSet & groundAnteriorCS = groundAnterior->upd_CoordinateSet();
    // flip
    double groundAnteriorCS0range[2] = {-Pi, Pi};
    groundAnteriorCS[0].setRange(groundAnteriorCS0range);
    groundAnteriorCS[0].setDefaultValue(0);
    groundAnteriorCS[0].setDefaultLocked(true);
    // u_integ (sprawl)
    double groundAnteriorCS1range[2] = {-Pi, Pi};
    groundAnteriorCS[1].setRange(groundAnteriorCS1range);
    groundAnteriorCS[1].setDefaultValue(0);
    groundAnteriorCS[1].setDefaultLocked(false);
    // roll
    double groundAnteriorCS2range[2] = {-3 * Pi, 3 * Pi};
    groundAnteriorCS[2].setRange(groundAnteriorCS2range);
    groundAnteriorCS[2].setDefaultValue(0);
    groundAnteriorCS[2].setDefaultLocked(true);
    // tx
    double groundAnteriorCS3range[2] = {-10, 10};
    groundAnteriorCS[3].setRange(groundAnteriorCS3range);
    groundAnteriorCS[3].setDefaultValue(0);
	groundAnteriorCS[3].setDefaultLocked(true);
    // ty
    double groundAnteriorCS4range[2] = {-1, 100};
    groundAnteriorCS[4].setRange(groundAnteriorCS4range);
    groundAnteriorCS[4].setDefaultValue(0);
	groundAnteriorCS[4].setDefaultLocked(false);
    // tz
    double groundAnteriorCS5range[2] = {-5, 5};
    groundAnteriorCS[5].setRange(groundAnteriorCS5range);
    groundAnteriorCS[5].setDefaultValue(0);
    groundAnteriorCS[5].setDefaultLocked(true);

    // Frame in which the X-axis points along ray K. K lies in the A1-A2
	// plane, so frame K is formed by rotation (by angle alpha) about A3.
    Body* frameK = newMasslessBody("K");
	Vec3 locationAKInAnterior(0);
	Vec3 orientationAKInAnterior(0);
	Vec3 locationAKinFrameK(0);
	Vec3 orientationAKinFrameK(0);
	PinJoint* anteriorFrameK = new PinJoint("anterior_K",
		*_anteriorBody, locationAKInAnterior, orientationAKInAnterior,
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
	Vec3 locationB2PostInFrameB2(0);
	Vec3 orientationB2PostInFrameB2(0, 0, -0.5 * Pi);
	Vec3 locationB2PostInPosterior(0);
	Vec3 orientationB2PostInPosterior(Pi, 0, 0.5 * Pi);
	PinJoint* FrameB2Posterior = new PinJoint("B2_posterior",
		*frameB2, locationB2PostInFrameB2, orientationB2PostInFrameB2,
		*_posteriorBody, locationB2PostInPosterior, orientationB2PostInPosterior, false);
	CoordinateSet& FrameB2PosteriorCS = FrameB2Posterior->upd_CoordinateSet();
	FrameB2PosteriorCS[0].setName("kane_v_integ");
	double FrameB2PosteriorCS0range[2] = {-Pi, Pi};
	FrameB2PosteriorCS[0].setRange(FrameB2PosteriorCS0range);
	FrameB2PosteriorCS[0].setDefaultValue(0);
	FrameB2PosteriorCS[0].setDefaultLocked(false);

	// TODO: add N, define gamma?

    // -- Add bodies.
    _cat.addBody(_anteriorBody);
	_cat.addBody(frameK);
	_cat.addBody(frameP);
	_cat.addBody(frameB2);
    _cat.addBody(_posteriorBody);
	//cat.addBody(frameN);

    // -- Add "no-twist" constraint. (TODO: is this right? don't we want to
    // constrain speeds?)
	CoordinateCouplerConstraint * twistConstr = new CoordinateCouplerConstraint();
    twistConstr->setName("twist");
    twistConstr->setIndependentCoordinateNames(Array<string>("kane_u_integ", 1));
    twistConstr->setDependentCoordinateName("kane_v_integ");
    Array<double> twistConstrFcnCoeff;
    twistConstrFcnCoeff.append(1);
    twistConstr->setFunction(new LinearFunction(twistConstrFcnCoeff));
	_cat.addConstraint(twistConstr);
}

void KaneScherFig2Modeling::addBaseForcesAndActuation()
{
    // This actuator is here to play around with the idea of using linear
    // actuators to actuate Kane and Scher's model. It is not actually part of
    // the Kane and Scher model itself.

    // Two springs connecting the anterior and posterior halves of
	// the cat, positioned symmetrically about its midline, act as
	// abdomen muscles.
	string body1Name = "anteriorBody";
	string body2Name = "posteriorBody";
    Vec3 point1L(-0.5 * _segmentalLength, 0, 0.5 * _segmentalDiam);
    Vec3 point2L(0.5 * _segmentalLength, 0, 0.5 * _segmentalDiam);
	Vec3 point1R(-0.5 * _segmentalLength, 0, -0.5 * _segmentalDiam);
    Vec3 point2R(0.5 * _segmentalLength, 0, -0.5 * _segmentalDiam);
    double stiffness = 1.0;
    double restlength = 0.75 * _segmentalLength;
    PointToPointSpring * absL = new PointToPointSpring(body1Name, point1L,
                                                       body2Name, point2L,
                                                       stiffness, restlength);
	PointToPointSpring * absR = new PointToPointSpring(body1Name, point1R,
                                                       body2Name, point2R,
                                                       stiffness, restlength);
    absL->setName("left_abs");
	absR->setName("right_abs");
	_cat.addForce(absL);
	_cat.addForce(absR);
}
