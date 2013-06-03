
#include "FelisCatusModeling.h"

int main(int argc, char *argv[])
{
    // A model that exhibits the counter-rotation mechanism of flipping
    // ========================================================================

    // Properties
    // ------------------------------------------------------------------------
    double segmentalLength = 0.175;		                   // m
    double segmentalDiam = 0.15;			               // m
    double segmentalMass = 1;				               // kg
    double segmentalTransverseMomentOfInertia = 1;         // kg-m^2
    // For actuators:
	double maxTorque = 40.0;                               // N-m

    // Ratio of transverse to axial moment of inertia:
    double JIratio = 0.25; // from Kane and Scher (1969)


    // Basics
    // ------------------------------------------------------------------------
    // Create the cat model.
    OpenSim::Model cat;

    // Name the cat after the founder of Stanford University.
    // This model will be able to hunch and wag (see the joints below).
    cat.setName("Leland_hunch_wag");

    // 'Turn off' gravity so it's easier to watch an animation of the flip.
    cat.setGravity(Vec3(0, 0, 0));


    // Anterior and posterior halves of the cat
    // ------------------------------------------------------------------------
    // Prepare inertia properties for the 2 primary segments of the cat.
    double segmentalAxialMoment = JIratio * segmentalTransverseMomentOfInertia;
    double Ixx = segmentalAxialMoment;
    double Iyy = segmentalTransverseMomentOfInertia;
    double Izz = segmentalTransverseMomentOfInertia;
    double Ixy = 0;
    double Ixz = 0;
    double Iyz = 0;
    Inertia segmentalInertia = OpenSim::Inertia(Ixx, Iyy, Izz, Ixz, Ixz, Iyz);

    // Anterior half of cat.
    Body * anteriorBody = new OpenSim::Body();
    anteriorBody->setName("anteriorBody");
    anteriorBody->setMass(segmentalMass);
    // By choosing the following as the mass center, we choose the origin of
    // the anteriorBody frame to be at its positive-X extent. That is, the
    // anterior body sits to the -X direction from its origin.
    anteriorBody->setMassCenter(Vec3(-0.5 * segmentalLength, 0, 0));
    anteriorBody->setInertia(segmentalInertia);

    // Posterior half of cat; same mass properties as anterior half.
    Body * posteriorBody = new OpenSim::Body();
    posteriorBody->setName("posteriorBody");
    posteriorBody->setMass(segmentalMass);
    // Posterior body sits to the +X direction from its origin.
    posteriorBody->setMassCenter(Vec3(0.5 * segmentalLength, 0, 0));
    posteriorBody->setInertia(segmentalInertia);


    // Joints
    // ------------------------------------------------------------------------
    Body & ground = _cat.getGroundBody();

    // Anterior body to the ground via a CustomJoint
    // `````````````````````````````````````````````
	// Rotation is defined via YZX Euler angles, named yaw, pitch, and
	// roll respectively. The important translation is in Y, the direction
	// of gravity.
	Vec3 locGAInGround(0);
    Vec3 orientGAInGround(0);
    Vec3 locGAInAnterior(0);
    Vec3 orientGAInAnterior(0);

    // To pass to the CustomJoint,farther down, a SpatialTransform:
    // The SpatialTransfrom has 6 transform axes. The first 3 are rotations,
    // defined about the axes of our choosing. The remaining 3 are translations,
    // which we choose to be along the X, Y, and Z directions of the ground's
    // frame.
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

    // Edit the Coordinate's created by the CustomJoint. The 6 coordinates
    // correspond to the TransformAxis's we set above.
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
    groundAnteriorCS[4].setDefaultValue(0);
	groundAnteriorCS[4].setDefaultLocked(false);
    // tz
    double groundAnteriorCS5range[2] = {-1, 1};
    groundAnteriorCS[5].setRange(groundAnteriorCS5range);
    groundAnteriorCS[5].setDefaultValue(0);
    groundAnteriorCS[5].setDefaultLocked(false);

    // Anterior to posterior body via a CustomJoint
    // ````````````````````````````````````````````
	// Rotation is defined via ZYX Euler angles.
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
    // There is no translation between the segments, and so we do not name the
    // remaining 3 TransformAxis's in the SpatialTransform.

    CustomJoint * anteriorPosterior = new CustomJoint("anterior_posterior",
            *anteriorBody, locAPInAnterior, orientAPInAnterior,
            *posteriorBody, locAPInPosterior, orientAPInPosterior,
			anteriorPosteriorST);

    // Set coordinate limits and default values from empirical data (i.e.,
    // photos & video).
	CoordinateSet & anteriorPosteriorCS = anteriorPosterior->upd_CoordinateSet();
    // hunch; [-20, +90] degrees
    double anteriorPosteriorCS0range[2] = {convertDegreesToRadians(-20),
										   convertDegreesToRadians(90)};
    anteriorPosteriorCS[0].setRange(anteriorPosteriorCS0range);
    anteriorPosteriorCS[0].setDefaultValue(convertDegreesToRadians(30));
    anteriorPosteriorCS[0].setDefaultLocked(false);
	// wag; [-45, 45] degrees
    double anteriorPosteriorCS1range[2] = {-0.25 * Pi, 0.25 * Pi};
    anteriorPosteriorCS[1].setRange(anteriorPosteriorCS1range);
    anteriorPosteriorCS[1].setDefaultValue(convertDegreesToRadians(-15));
    anteriorPosteriorCS[1].setDefaultLocked(false);
	// twist; [-80, 80] degrees
    double anteriorPosteriorCS2range[2] = {convertDegreesToRadians(-80),
										   convertDegreesToRadians(80)};
    anteriorPosteriorCS[2].setRange(anteriorPosteriorCS2range);
    anteriorPosteriorCS[2].setDefaultValue(0);
    // This model can't twist; we'll unlock this for the next model.
    anteriorPosteriorCS[2].setDefaultLocked(true);

    // Add bodies to the model
    // ------------------------------------------------------------------------
    // ...now that we have connected the bodies via joints.
    cat.addBody(anteriorBody);
    cat.addBody(posteriorBody);


    // Display goemetry
    // ------------------------------------------------------------------------
    // So that we can see what the cat's up to.
    // By default, the cylinder has a diameter of 1 meter, height of 1 meter,
    // its centroid is at (0, 0, 0) (its origin), and its axis of symmetry is
    // its Y axis.

    // Anterior body
    // `````````````
    // 'cylinder.vtp' is in the Geometry folder of an OpenSim installation.
    DisplayGeometry * anteriorDisplay = new DisplayGeometry("cylinder.vtp");
    anteriorDisplay->setOpacity(0.5);
    anteriorDisplay->setColor(Vec3(0.5, 0.5, 0.5));

    // We want the centroid to be at (-0.5 * segmentalLength, 0, 0), and for
    // its axis of symmetry to be its body's (the body it's helping us to
    // visualize) Y axis.
    Rotation rot;
    // We align the cylinder's symmetry (Y) axis with the body's X axis:
    rot.setRotationFromAngleAboutZ(0.5 * Pi);
    anteriorDisplay->setTransform(
            Transform(rot, Vec3(-0.5 * segmentalLength, 0, 0)));
    anteriorDisplay->setScaleFactors(
            Vec3(_segmentalDiam, segmentalLength, segmentalDiam));
    anteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(anteriorDisplay);
    anteriorBody->updDisplayer()->setShowAxes(true);

    // Posterior body
    // ``````````````
    DisplayGeometry * posteriorDisplay = new DisplayGeometry("cylinder.vtp");
    posteriorDisplay->setOpacity(0.5);
    posteriorDisplay->setColor(Vec3(0.7, 0.7, 0.7));
    // We specify the desired location of the cylinder's centroid:
    posteriorDisplay->setTransform(
            Transform(rot, Vec3(0.5 * segmentalLength, 0, 0)));
    posteriorDisplay->setScaleFactors(
            Vec3(segmentalDiam, segmentalLength, segmentalDiam));
    posteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(posteriorDisplay);
    posteriorBody->updDisplayer()->setShowAxes(true);


    // Actuation
    // ------------------------------------------------------------------------
    // Since these coordinates are angles, the actuators are effectively torque
    // actuators. The reason to use a CoordinateActuator instead of a
    // TorqueActuator is that we needn't specify the axis of the actuation, or
    // the bodies on which it acts.

    // hunch
    CoordinateActuator * hunchAct = new CoordinateActuator("hunch");
    hunchAct->setName("hunch_actuator");
    hunchAct->setMinControl(-maxTorque);
    hunchAct->setMaxControl(maxTorque);
    cat.addForce(hunchAct);

    // wag
    CoordinateActuator * wagAct = new CoordinateActuator("wag");
    wagAct->setName("wag_actuator");
    wagAct->setMinControl(-maxTorque);
    wagAct->setMaxControl(maxTorque);
    cat.addForce(wagAct);


    // Print the model
    // ------------------------------------------------------------------------
    cat.print("flippinfelines_hunch_wag.osim");


    // Second model: adding legs for the variable inertia mechanism of flipping
    // ========================================================================
    // This model will additionally be able to twist, and has legs.
    cat.setName("Leland_hunch_wag_twist_legs");

    // Allow twist.
    anteriorPosteriorCS[2].setDefaultLocked(false);

    // Leg properties
    // ------------------------------------------------------------------------
    double legsLength = 0.125;                             // m
    double legsDiam = 0.1 * _legsLength;                   // m
    // Sum of both legs (60% distance across the belly):
    double legsWidth = 0.6 * _segmentalDiam;               // m
    double legsMass = 0.2;                                 // kg

    // Leg bodies
    // ------------------------------------------------------------------------

    // Leg joints
    // ------------------------------------------------------------------------

    return EXIT_SUCCESS;
};
