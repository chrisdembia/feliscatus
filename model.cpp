#include <iostream>

#include <OpenSim/OpenSim.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using OpenSim::Array;
using OpenSim::Body;
using OpenSim::CoordinateSet;
using OpenSim::CustomJoint;
using OpenSim::DisplayGeometry;
using OpenSim::PinJoint;
using OpenSim::Model;
using OpenSim::SpatialTransform;

using SimTK::Inertia;
using SimTK::Pi;
using SimTK::Rotation;
using SimTK::Transform;
using SimTK::Vec3;

/**
 * Creates an OpenSim model of a cat, sufficient for exploring the cat's
 * righting reflex.
 * */
int main()
{
    // We use the general convention that body origins are at joint locations,
    // and thus mass centers are specified to be at some further location.
    // Furthermore, TODO about "pre-rotations".

	// Numbers.
	double bodyLength = 1; // m
	double bodyDiam = 1; // m
	double mass = 1; // kg
	double Ixx = 1; // kg-m^2
	double Iyy = 1; // kg-m^2
	double Izz = 1; // kg-m^2
	double Ixy = 0; // kg-m^2
	double Ixz = 0; // kg-m^2
	double Iyz = 0; // kg-m^2
	// TODO is inertia about center of mass?
	Inertia inertia = Inertia(Ixx, Iyy, Izz, Ixz, Ixz, Iyz);

    // Create the model.
    Model cat = Model();
    cat.setName("feliscatus");
    // Needed for specifying joint(s).
    Body & ground = cat.getGroundBody();
	ground.addDisplayGeometry("treadmill.vtp");

	// ----- Body construction and mass properties.

    // Anterior half of cat.
    Body * anteriorBody = new Body();
	anteriorBody->setName("anteriorBody");
	anteriorBody->setMass(mass);
	anteriorBody->setMassCenter(Vec3(0.5 * bodyDiam, 0.0, 0.0));
	anteriorBody->setInertia(inertia);

	// Posterior half of cat.
	Body * posteriorBody = new Body();
	posteriorBody->setName("posteriorBody");
	posteriorBody->setMass(mass);
	posteriorBody->setMassCenter(Vec3(-0.5 * bodyLength, 0.0, 0.0));
	posteriorBody->setInertia(inertia);

	// ----- Joints.

    // --- Joint between the ground and the anterior body.

	// -- This joint is a CustomJoint, which requires a SpatialTransform.
	// This construction goes smoothly seemingly only if we define the SpatialTransform
	// first, and pass it to the CustomJoint as a constructor argument.
	// This was gleaned by reading the source code, particularly
	// CustomJoint::constructCoordinates().
	// We are following the example of a CustomJoint given by gait2354.
	SpatialTransform groundAnteriorST;

	// Rotation. ZXY.
	groundAnteriorST.updTransformAxis(0).setCoordinateNames(
		Array<string>("anterior_flip", 1));
	groundAnteriorST.updTransformAxis(0).setAxis(Vec3(0, 0, 1));
	
	groundAnteriorST.updTransformAxis(1).setCoordinateNames(
		Array<string>("anterior_sprawl", 1));
	groundAnteriorST.updTransformAxis(1).setAxis(Vec3(1, 0, 0));

	groundAnteriorST.updTransformAxis(2).setCoordinateNames(
		Array<string>("anterior_roll", 1));
	groundAnteriorST.updTransformAxis(2).setAxis(Vec3(0, 1, 0));

	// Translation.
	groundAnteriorST.updTransformAxis(3).setCoordinateNames(
		Array<string>("anterior_tx", 1));
	groundAnteriorST.updTransformAxis(3).setAxis(Vec3(1, 0, 0));
	groundAnteriorST.updTransformAxis(4).setCoordinateNames(
		Array<string>("anterior_ty", 1));
	groundAnteriorST.updTransformAxis(4).setAxis(Vec3(0, 1, 0));
	groundAnteriorST.updTransformAxis(5).setCoordinateNames(
		Array<string>("anterior_tz", 1));
	groundAnteriorST.updTransformAxis(5).setAxis(Vec3(0, 0, 1));

	// -- The actual joint definition.
    Vec3 locGAInGround(0);
    Vec3 orientGAInGround(0);

    // This joint is at the same location (in any frame) as the
    // anterior_posterior joint defined below.
    Vec3 locGAInAnterior(0);
    Vec3 orientGAInAnterior(0);
    CustomJoint * groundAnterior = new CustomJoint("ground_anterior",
            ground, locGAInGround, orientGAInGround,
            *anteriorBody, locGAInAnterior, orientGAInAnterior, groundAnteriorST);

    // Set properties of the joint's coordinates.
    CoordinateSet & groundAnteriorCS = groundAnterior->upd_CoordinateSet();

	// flip
	double groundAnteriorCS0range[2] = {-Pi, Pi};
	groundAnteriorCS[0].setRange(groundAnteriorCS0range);
	groundAnteriorCS[0].setDefaultValue(0.0);
	groundAnteriorCS[0].setDefaultLocked(false);

	// sprawl
	double groundAnteriorCS1range[2] = {-Pi, Pi};
	groundAnteriorCS[1].setRange(groundAnteriorCS1range);
	groundAnteriorCS[1].setDefaultValue(0.0);
	groundAnteriorCS[1].setDefaultLocked(false);
	
	// roll
	double groundAnteriorCS2range[2] = {-3 * Pi, 3 * Pi};
	groundAnteriorCS[2].setRange(groundAnteriorCS2range);
	groundAnteriorCS[2].setDefaultValue(0.0);
	groundAnteriorCS[2].setDefaultLocked(false);

	// tx
	double groundAnteriorCS3range[2] = {-10, 10};
	groundAnteriorCS[3].setRange(groundAnteriorCS3range);
	groundAnteriorCS[3].setDefaultValue(0.0);

	// ty
	double groundAnteriorCS4range[2] = {-1, 100};
	groundAnteriorCS[4].setRange(groundAnteriorCS4range);
	groundAnteriorCS[4].setDefaultValue(20);

	// tz
	double groundAnteriorCS5range[2] = {-5, 5};
	groundAnteriorCS[5].setRange(groundAnteriorCS5range);
	groundAnteriorCS[5].setDefaultValue(0.0);
	groundAnteriorCS[5].setDefaultLocked(false);
        
	// -- Joint between the anterior and posterior bodies.
	// TODO there is a u-joint class
	Vec3 locAPInAnterior(0);
    Vec3 orientAPInAnterior(0, 0, 0); // TODO use Pi, or just place geom elsewhere?
    Vec3 locAPInPosterior(0);
    Vec3 orientAPInPosterior(0);
    PinJoint * anteriorPosterior = new PinJoint("anterior_posterior",
            *anteriorBody, locAPInAnterior, orientAPInAnterior,
            *posteriorBody, locAPInPosterior, orientAPInPosterior);
    // Joint coordinates.
	// TODO would prefer that these rotations could be "body-fixed".
    CoordinateSet & anteriorPosteriorCS = anteriorPosterior->upd_CoordinateSet();
	anteriorPosteriorCS[0].setName("waist_hunch");
	double anteriorPosteriorCS0range[2] = {-Pi, Pi};
	anteriorPosteriorCS[0].setRange(anteriorPosteriorCS0range);
	anteriorPosteriorCS[0].setDefaultValue(0.0);
	anteriorPosteriorCS[0].setDefaultLocked(false);


	// ----- Geometry.
    DisplayGeometry * anteriorDisplay = 
        new DisplayGeometry(
                "feliscatus_cylinder_with_two_offset_feet_nubs.obj");
    anteriorDisplay->setOpacity(0.5);
    anteriorDisplay->setColor(Vec3(0.5, 0.5, 0.5));
    anteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(
            anteriorDisplay);
    anteriorBody->updDisplayer()->setShowAxes(true);

    DisplayGeometry * posteriorDisplay = 
        new DisplayGeometry(
                "feliscatus_cylinder_with_two_offset_feet_nubs.obj");
    posteriorDisplay->setOpacity(0.5);
    posteriorDisplay->setColor(Vec3(0.7, 0.7, 0.7));
    Rotation rot;
    rot.setRotationFromAngleAboutY(Pi);
    posteriorDisplay->setTransform(Transform(rot));
    posteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(
            posteriorDisplay);
    posteriorBody->updDisplayer()->setShowAxes(true);   

    
    

    // ---- Coordinate limits.
    // TODO 

    // -- Add bodies to model.
    cat.addBody(anteriorBody);
	cat.addBody(posteriorBody);

    cat.print("feliscatus.osim");

    // Memory management.
	// Doing this deletion ourselves causes a segfault.
    // delete anteriorBody;
    // anteriorBody = NULL;

    return EXIT_SUCCESS;
}
