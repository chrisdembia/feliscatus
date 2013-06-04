
#include <OpenSim/OpenSim.h>

/**
 * Creates two cat models for flipping. The first is simply two segments
 * connected by a 2-degree-of-freedom (no twist) joint. It exhibits the
 * counter-rotation mechanism of flipping. The second adds actuate-able
 * legs to the first model, as well as unlocking the twist degree of
 * freedom between the model's two halves. This model exhibits the variable-
 * inertia mechanism of flipping.
 * */
int main(int argc, char *argv[])
{
    // First model: exhibiting the counter-rotation mechanism of flipping
    // ========================================================================

    // Properties
    // ------------------------------------------------------------------------
    double segmentalLength = 0.175;		                   // m
    double segmentalDiam = 0.15;			               // m
    double segmentalMass = 1;				               // kg
    double segmentalTransverseMomentOfInertia = 1;         // kg-m^2
    
    // Ratio of transverse to axial moment of inertia (Kane and Scher, 1969):
    double JIratio = 0.25;
    
    // For actuators:
	double maxTorque = 40.0;                               // N-m


    // Basics
    // ------------------------------------------------------------------------
    // Create the cat model.
    OpenSim::Model cat;

    // Name the cat after the founder of Stanford University.
    // This model will be able to hunch and wag (see the joints below).
    cat.setName("Leland_hunch_wag");

    // 'Turn off' gravity so it's easier to watch an animation of the flip.
    using SimTK::Vec3;
    cat.setGravity(Vec3(0, 0, 0));


    // Define anterior and posterior halves of the cat
    // ------------------------------------------------------------------------
    // Prepare inertia properties for the 2 primary segments of the cat.
    double segmentalAxialMomentOfInertia = JIratio * segmentalTransverseMomentOfInertia;
    double Ixx = segmentalAxialMomentOfInertia;
    double Iyy = segmentalTransverseMomentOfInertia;
    double Izz = segmentalTransverseMomentOfInertia;
    double Ixy = 0;
    double Ixz = 0;
    double Iyz = 0;
    using SimTK::Inertia;
    Inertia segmentalInertia = Inertia(Ixx, Iyy, Izz, Ixz, Ixz, Iyz);

    // Anterior half of cat.
    using OpenSim::Body;
    Body * anteriorBody = new Body();
    anteriorBody->setName("anteriorBody");
    anteriorBody->setMass(segmentalMass);
    // By choosing the following as the mass center, we choose the origin of
    // the anteriorBody frame to be at the body's positive-X extent. That is,
    // the anterior body sits to the -X direction from its origin.
    anteriorBody->setMassCenter(Vec3(-0.5 * segmentalLength, 0, 0));
    anteriorBody->setInertia(segmentalInertia);

    // Posterior half of cat (same mass properties as anterior half).
    Body * posteriorBody = new Body();
    posteriorBody->setName("posteriorBody");
    posteriorBody->setMass(segmentalMass);
    // Posterior body sits to the +X direction from its origin.
    posteriorBody->setMassCenter(Vec3(0.5 * segmentalLength, 0, 0));
    posteriorBody->setInertia(segmentalInertia);


    // Define joints between bodies (ground and two halves)
    // ------------------------------------------------------------------------
    Body & ground = cat.getGroundBody();

    // Anterior body to the ground via a CustomJoint
    // `````````````````````````````````````````````
    using OpenSim::CustomJoint;
	// Rotation is defined via YZX Euler angles, named yaw, pitch, and
	// roll respectively.
	Vec3 locGAInGround(0);
    Vec3 orientGAInGround(0);
    Vec3 locGAInAnterior(0);
    Vec3 orientGAInAnterior(0);

    // To pass to the CustomJoint (farther down), define a SpatialTransform.
    // The SpatialTransfrom has 6 transform axes. The first 3 are rotations,
    // defined about the axes of our choosing. The remaining 3 are translations,
    // which we choose to be along the X, Y, and Z directions of the ground's
    // frame.
    using OpenSim::Array;
    OpenSim::SpatialTransform groundAnteriorST;
    groundAnteriorST.updTransformAxis(0).setCoordinateNames(
            Array<std::string>("yaw", 1));
    groundAnteriorST.updTransformAxis(0).setAxis(Vec3(0, 1, 0));

    groundAnteriorST.updTransformAxis(1).setCoordinateNames(
            Array<std::string>("pitch", 1));
    groundAnteriorST.updTransformAxis(1).setAxis(Vec3(0, 0, 1));

    groundAnteriorST.updTransformAxis(2).setCoordinateNames(
            Array<std::string>("roll", 1));
    groundAnteriorST.updTransformAxis(2).setAxis(Vec3(1, 0, 0));

    groundAnteriorST.updTransformAxis(3).setCoordinateNames(
            Array<std::string>("tx", 1));
    groundAnteriorST.updTransformAxis(3).setAxis(Vec3(1, 0, 0));

    groundAnteriorST.updTransformAxis(4).setCoordinateNames(
            Array<std::string>("ty", 1));
    groundAnteriorST.updTransformAxis(4).setAxis(Vec3(0, 1, 0));

    groundAnteriorST.updTransformAxis(5).setCoordinateNames(
            Array<std::string>("tz", 1));
    groundAnteriorST.updTransformAxis(5).setAxis(Vec3(0, 0, 1));

    CustomJoint * groundAnterior = new CustomJoint("ground_anterior",
            ground, locGAInGround, orientGAInGround,
            *anteriorBody, locGAInAnterior, orientGAInAnterior,
            groundAnteriorST);

    // Edit the Coordinate's created by the CustomJoint. The 6 coordinates
    // correspond to the TransformAxis's we set above.
    using OpenSim::CoordinateSet;
    using SimTK::convertDegreesToRadians;
    using SimTK::Pi;
    CoordinateSet & groundAnteriorCS = groundAnterior->upd_CoordinateSet();
    // yaw
    // As is, the range only affects how one can vary this coordinate in the
    // GUI. The range is not a joint limit, and does not affect dynamics.
    double groundAnteriorCS0range[2] = {-Pi, Pi};
    groundAnteriorCS[0].setRange(groundAnteriorCS0range);
    groundAnteriorCS[0].setDefaultValue(0);
    groundAnteriorCS[0].setDefaultLocked(false);
    // pitch
    double groundAnteriorCS1range[2] = {-Pi, Pi};
    groundAnteriorCS[1].setRange(groundAnteriorCS1range);
    groundAnteriorCS[1].setDefaultValue(convertDegreesToRadians(-15));
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

    OpenSim::SpatialTransform anteriorPosteriorST;
	anteriorPosteriorST.updTransformAxis(0).setCoordinateNames(
            Array<std::string>("hunch", 1));
    anteriorPosteriorST.updTransformAxis(0).setAxis(Vec3(0, 0, 1));

    anteriorPosteriorST.updTransformAxis(1).setCoordinateNames(
            Array<std::string>("wag", 1));
    anteriorPosteriorST.updTransformAxis(1).setAxis(Vec3(0, 1, 0));

    anteriorPosteriorST.updTransformAxis(2).setCoordinateNames(
            Array<std::string>("twist", 1));
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
    // hunch: [-20, +90] degrees
    double anteriorPosteriorCS0range[2] = {convertDegreesToRadians(-20),
										   convertDegreesToRadians(90)};
    anteriorPosteriorCS[0].setRange(anteriorPosteriorCS0range);
    anteriorPosteriorCS[0].setDefaultValue(convertDegreesToRadians(30));
    anteriorPosteriorCS[0].setDefaultLocked(false);
	// wag: [-45, 45] degrees
    double anteriorPosteriorCS1range[2] = {-0.25 * Pi, 0.25 * Pi};
    anteriorPosteriorCS[1].setRange(anteriorPosteriorCS1range);
    anteriorPosteriorCS[1].setDefaultValue(0);
    anteriorPosteriorCS[1].setDefaultLocked(false);
	// twist: [-80, 80] degrees
    double anteriorPosteriorCS2range[2] = {convertDegreesToRadians(-80),
										   convertDegreesToRadians(80)};
    anteriorPosteriorCS[2].setRange(anteriorPosteriorCS2range);
    anteriorPosteriorCS[2].setDefaultValue(0);
    // This first model can't twist; we'll unlock this for the next model.
    anteriorPosteriorCS[2].setDefaultLocked(true);


    // Add bodies to the model
    // ------------------------------------------------------------------------
    // ...now that we have connected the bodies via joints.
    cat.addBody(anteriorBody);
    cat.addBody(posteriorBody);


    // Display geometry
    // ------------------------------------------------------------------------
    using OpenSim::DisplayGeometry;
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
    // its axis of symmetry to be its body's (i.e., the body it's helping us
    // to visualize) X axis.
    SimTK::Rotation rot;
    // Rotate the cylinder's symmetry (Y) axis to align with the body's X axis:
    rot.setRotationFromAngleAboutZ(0.5 * Pi);
    // Tranform combines rotation and translation:
    anteriorDisplay->setTransform(
            SimTK::Transform(rot, Vec3(-0.5 * segmentalLength, 0, 0)));
    anteriorDisplay->setScaleFactors(
            Vec3(segmentalDiam, segmentalLength, segmentalDiam));
    anteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(anteriorDisplay);
    anteriorBody->updDisplayer()->setShowAxes(true);

    // Posterior body
    // ``````````````
    DisplayGeometry * posteriorDisplay = new DisplayGeometry("cylinder.vtp");
    posteriorDisplay->setOpacity(0.5);
    posteriorDisplay->setColor(Vec3(0.7, 0.7, 0.7));
    
    posteriorDisplay->setTransform(
            SimTK::Transform(rot, Vec3(0.5 * segmentalLength, 0, 0)));
    posteriorDisplay->setScaleFactors(
            Vec3(segmentalDiam, segmentalLength, segmentalDiam));
    posteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(posteriorDisplay);
    posteriorBody->updDisplayer()->setShowAxes(true);


    // Actuation
    // ------------------------------------------------------------------------
    // Since the coordinates between the segments are angles, the actuators are
    // effectively torque actuators. The reason to use a CoordinateActuator
    // instead of a TorqueActuator is that for a CoordinateActuator we needn't
    // specify the axis of the actuation, or the bodies on which it acts.
    using OpenSim::CoordinateActuator;

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
    
    
    

    // Second model: adding legs for the variable-inertia mechanism of flipping
    // ========================================================================
    // NOTE: Most of the set-up for this model has already been done. Many of
    // ----  the variables defined above are simply updated or renamed below.
    
    // This model will additionally be able to twist, and has legs.
    cat.setName("Leland_hunch_wag_twist_legs");

    // Allow twist.
    anteriorPosteriorCS[2].setDefaultLocked(false);

    CoordinateActuator * twistAct = new CoordinateActuator("twist");
    twistAct->setName("twist_actuator");
    twistAct->setMinControl(-maxTorque);
    twistAct->setMaxControl(maxTorque);
    cat.addForce(twistAct);
    
    
    // Leg properties
    // ------------------------------------------------------------------------
    double legsLength = 0.125;                             // m
    double legsDiam = 0.1 * legsLength;                    // m
    // Sum of both legs (60% distance across the belly):
    double legsWidth = 0.6 * segmentalDiam;                // m
    double legsMass = 0.2;                                 // kg
    
    
    // Define leg bodies
    // ------------------------------------------------------------------------
    // Scale the segmental inertia.
    Inertia legsInertia = (legsMass/segmentalMass) * segmentalInertia;

    // Anterior and posterior legs.
    Body * anteriorLegs = new Body();
    anteriorLegs->setName("anteriorLegs");
    anteriorLegs->setMass(legsMass);
    anteriorLegs->setMassCenter(Vec3(0.5 * legsLength, 0, 0));
    anteriorLegs->setInertia(legsInertia);

    Body * posteriorLegs = new Body();
    posteriorLegs->setName("posteriorLegs");
    posteriorLegs->setMass(legsMass);
    posteriorLegs->setMassCenter(Vec3(0.5 * legsLength, 0, 0));
    posteriorLegs->setInertia(legsInertia);
    
    
    // Define leg joints (i.e., between legs and two halves of cat)
    // ------------------------------------------------------------------------
    using OpenSim::PinJoint;

    // Anterior leg
    // ````````````
	Vec3 locALegsInAnterior(-0.75 * segmentalLength, 0.5 * segmentalDiam, 0);
    Vec3 orientALegsInAnterior(0);
    Vec3 locALegsInLegs(0);
    // Rotate the leg about what will become the pin-joint axis (the leg's
    // Z axis) so that it points straight out from the belly when leg angle
    // is 0. In other words, position the leg's long (X) axis normal to the
    // half of the cat.
    Vec3 orientALegsInLegs(0, 0, -0.5 * Pi);
    
    PinJoint * anteriorToLegs = new PinJoint("anterior_legs",
            *anteriorBody, locALegsInAnterior, orientALegsInAnterior,
            *anteriorLegs, locALegsInLegs, orientALegsInLegs);
    CoordinateSet & anteriorToLegsCS = anteriorToLegs->upd_CoordinateSet();
    anteriorToLegsCS[0].setName("frontLegs");
    double anteriorToLegsCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    anteriorToLegsCS[0].setRange(anteriorToLegsCS0range);

    // So that the legs are directed strictly upwards initially:
    double pitch = groundAnteriorCS[1].getDefaultValue();
    anteriorToLegsCS[0].setDefaultValue(-pitch);
    anteriorToLegsCS[0].setDefaultLocked(false);

    // Posterior leg
    // `````````````
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
    
    
    // Add bodies to the model
    // ------------------------------------------------------------------------
    // ...now that we have connected the bodies via joints.
    cat.addBody(anteriorLegs);
    cat.addBody(posteriorLegs);
    
    
    // Display goemetry
    // ------------------------------------------------------------------------
    // Both legs have the same display geometry.

    // 'box.vtp' is in the Geometry folder of an OpenSim installation.
    DisplayGeometry legsDisplay = DisplayGeometry("box.vtp");
    legsDisplay.setOpacity(0.5);
    legsDisplay.setColor(Vec3(0.7, 0.7, 0.7));
    legsDisplay.setTransform(Transform(Vec3(0.3 * legsLength, 0, 0)));
    legsDisplay.setScaleFactors(Vec3(legsLength, legsDiam, legsWidth));

    anteriorLegs->updDisplayer()->updGeometrySet().cloneAndAppend(legsDisplay);
    anteriorLegs->updDisplayer()->setShowAxes(true);

    posteriorLegs->updDisplayer()->updGeometrySet().cloneAndAppend(legsDisplay);
    posteriorLegs->updDisplayer()->setShowAxes(true);
    

    // Actuation
    // ------------------------------------------------------------------------
    // front legs.
    CoordinateActuator * frontLegsAct = new CoordinateActuator("frontLegs");
    frontLegsAct->setName("frontLegs_actuator");
    frontLegsAct->setMinControl(-maxTorque);
    frontLegsAct->setMaxControl(maxTorque);
    cat.addForce(frontLegsAct);

    // back legs.
    CoordinateActuator * backLegsAct = new CoordinateActuator("backLegs");
    backLegsAct->setName("backLegs_actuator");
    backLegsAct->setMinControl(-maxTorque);
    backLegsAct->setMaxControl(maxTorque);
    cat.addForce(backLegsAct);

    
    // Enforce joint limits on the legs
    // ------------------------------------------------------------------------
    using OpenSim::CoordinateLimitForce;

    CoordinateLimitForce * frontLegsLimitForce = new CoordinateLimitForce(
                "frontLegs", 90, 1.0E2, -90, 1.0E2, 1.0E1, 2.0, false);
	cat.addForce(frontLegsLimitForce);

    CoordinateLimitForce * backLegsLimitForce = new CoordinateLimitForce(
                "backLegs", 90, 1.0E2, -90, 1.0E2, 1.0E1, 2.0, false);
	cat.addForce(backLegsLimitForce);
    
    
    // Print the model
    // ------------------------------------------------------------------------
    cat.print("flippinfelines_hunch_wag_twist_legs.osim");

    return EXIT_SUCCESS;
};
