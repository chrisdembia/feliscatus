#include <iostream>

#include <OpenSim/OpenSim.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using OpenSim::Body;
using OpenSim::CoordinateSet;
using OpenSim::CustomJoint;
using OpenSim::Model;

using SimTK::Vec3;

int main()
{
    // Create the model.
    Model cat = Model();
    cat.setName("feliscatus");
    // Needed for specifying joint(s).
    Body & ground = cat.getGroundBody();

    // Anterior half of cat.
    Body * anteriorBody = new Body();
    anteriorBody->setName("anteriorBody");
    anteriorBody->addDisplayGeometry("treadmill.vtp");

    // Joint between the ground and the anterior segment.
    Vec3 locGAInGround(0);
    Vec3 orientGAInGround(0);
    Vec3 locGAInAnterior(0);
    Vec3 orientGAInAnterior(0);
    CustomJoint * groundAnterior = new CustomJoint("ground_anterior",
            ground, locGAInGround, orientGAInGround,
            *anteriorBody, locGAInAnterior, orientGAInAnterior);
    // Joint coordinates.
    CoordinateSet & groundAnteriorCS = groundAnterior->upd_CoordinateSet();
    groundAnteriorCS[0].setName("tilt");
    groundAnteriorCS[1].setName("yaw");
    groundAnteriorCS[2].setName("pitch");

    // Add bodies to model.
    cat.addBody(anteriorBody);

    cat.print("feliscatus.osim");

    // Memory management.
    // delete anteriorBody;
    // anteriorBody = NULL;

    return EXIT_SUCCESS;
}
