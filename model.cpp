#include <iostream>

#include <OpenSim/OpenSim.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using OpenSim::Body;
using OpenSim::CoordinateSet;
using OpenSim::CustomJoint;
using OpenSim::PinJoint;
using OpenSim::Model;
using OpenSim::SpatialTransform;

using SimTK::Inertia;
using SimTK::Pi;
using SimTK::Vec3;

int main()
{

    // We use the general convention that body origins are at joint locations,
    // and thus mass centers are specified to be at some further location.

	// Numbers.
	double mass = 1;
	double Ixx = 1;
	double Iyy = 1;
	double Izz = 1;
	double Ixy = 0;
	double Ixz = 0;
	double Iyz = 0;
	// TODO is inertia about center of mass?
	Inertia inertia = Inertia(Ixx, Iyy, Izz, Ixz, Ixz, Iyz);

    // Create the model.
    Model cat = Model();
    cat.setName("feliscatus");
    // Needed for specifying joint(s).
    Body & ground = cat.getGroundBody();
	ground.addDisplayGeometry("treadmill.vtp");

    // Anterior half of cat.
    Body * anteriorBody = new Body();
	anteriorBody->setMass(mass);
	anteriorBody->setMassCenter(Vec3(0.0, 0.0, 0.0));
	anteriorBody->setInertia(inertia);

    anteriorBody->setName("anteriorBody");
    anteriorBody->addDisplayGeometry(
            "feliscatus_cylinder_with_two_offset_feet_nubs.obj");

    // Joint between the ground and the anterior body.
    Vec3 locGAInGround(0);
    Vec3 orientGAInGround(0);
    // This joint is at the same location (in any frame) as the
    // anterior_posterior joint defined below.
    Vec3 locGAInAnterior(0);
    Vec3 orientGAInAnterior(0);
    CustomJoint * groundAnterior = new CustomJoint("ground_anterior",
            ground, locGAInGround, orientGAInGround,
            *anteriorBody, locGAInAnterior, orientGAInAnterior);
	SpatialTransform * groundAnteriorST = new SpatialTransform();
	groundAnteriorST->connectToJoint(*groundAnterior);
    groundAnteriorST->
	groundAnteriorST->constructIndependentAxes(6, 0);
	// // This gets the first TransformAxis.
	// cout << "COORD AXIS NAMES: " << groundAnteriorST[0].getCoordinateNames() << endl;

    // Joint coordinates.
    //CoordinateSet & groundAnteriorCS = groundAnterior->upd_CoordinateSet();

	groundAnteriorCS[0].setName("anterior_flail");
	double groundAnteriorCS0range[2] = {-Pi, Pi};
	groundAnteriorCS[0].setRange(groundAnteriorCS0range);
	groundAnteriorCS[0].setDefaultValue(0.0);
	groundAnteriorCS[0].setDefaultLocked(false);

	groundAnteriorCS[1].setName("anterior_sprawl");
	double groundAnteriorCS1range[2] = {-Pi, Pi};
	groundAnteriorCS[1].setRange(groundAnteriorCS1range);
	groundAnteriorCS[1].setDefaultValue(0.0);
	groundAnteriorCS[1].setDefaultLocked(false);

	groundAnteriorCS[2].setName("anterior_roll");
	double groundAnteriorCS2range[2] = {-3 * Pi, 3 * Pi};
	groundAnteriorCS[2].setRange(groundAnteriorCS2range);
	groundAnteriorCS[2].setDefaultValue(0.0);
	groundAnteriorCS[2].setDefaultLocked(false);

	groundAnteriorCS[3].setName("anterior_tx");
	double groundAnteriorCS3range[2] = {-10, 10};
	groundAnteriorCS[3].setRange(groundAnteriorCS3range);
	groundAnteriorCS[3].setDefaultValue(0.0);

	groundAnteriorCS[4].setName("anterior_ty");
	double groundAnteriorCS4range[2] = {-1, 100};
	groundAnteriorCS[4].setRange(groundAnteriorCS4range);
	groundAnteriorCS[4].setDefaultValue(20);

	groundAnteriorCS[5].setName("anterior_tz");
	double groundAnteriorCS5range[2] = {-5, 5};
	groundAnteriorCS[5].setRange(groundAnteriorCS5range);
	groundAnteriorCS[5].setDefaultValue(0.0);
	groundAnteriorCS[5].setDefaultLocked(false);

	// Posterior half of cat.
	Body * posteriorBody = new Body();
	posteriorBody->setMass(mass);
	posteriorBody->setMassCenter(Vec3(0.0, 0.0, 0.0));
	posteriorBody->setInertia(inertia);

	posteriorBody->setName("posteriorBody");
    posteriorBody->addDisplayGeometry("feliscatus_cylinder_with_two_offset_feet_nubs.obj");
	// TODO rotate around. posteriorBody->updDisplayer()->setTransform();

	// Joint between the anterior and posterior bodies.
	Vec3 locAPInAnterior(0);
    Vec3 orientAPInAnterior(0);
    Vec3 locAPInPosterior(0);
    Vec3 orientAPInPosterior(0);
    PinJoint * anteriorPosterior = new PinJoint("anterior_posterior",
            *anteriorBody, locAPInAnterior, orientAPInAnterior,
            *posteriorBody, locAPInPosterior, orientAPInPosterior);
    // Joint coordinates.
	// TODO would prefer that these rotations could be "body-fixed".
    CoordinateSet & anteriorPosteriorCS = anteriorPosterior->upd_CoordinateSet();
	anteriorPosteriorCS[0].setName("anterior_rx");
	double anteriorPosteriorCS0range[2] = {-Pi, Pi};
	anteriorPosteriorCS[0].setRange(anteriorPosteriorCS0range);
	anteriorPosteriorCS[0].setDefaultValue(0.0);
	anteriorPosteriorCS[0].setDefaultLocked(false);

    // Add bodies to model.
    cat.addBody(anteriorBody);
	cat.addBody(posteriorBody);

    cat.print("feliscatus.osim");

    // Memory management.
    // delete anteriorBody;
    // anteriorBody = NULL;

    return EXIT_SUCCESS;
}
