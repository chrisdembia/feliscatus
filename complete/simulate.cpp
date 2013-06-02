#include <iostream>

#include <OpenSim/OpenSim.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using OpenSim::CoordinateSet;
using OpenSim::Manager;
using OpenSim::Model;
using OpenSim::ModelVisualizer;

using SimTK::Pi;
using SimTK::RungeKuttaMersonIntegrator;
using SimTK::Stage;
using SimTK::State;
using SimTK::Vec3;
using SimTK::Visualizer;

/**
 * Simulates a cat model using the Simbody visualizer.
 * */
int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        cout << "The single input to this exectuable is the name "
            "of the model file to simulate. For example: "
            "$ simulate feliscatus.osim" << endl;
        return EXIT_SUCCESS;
    }

    char * modelFileName = argv[1];

    // Open the model.
    Model cat(modelFileName);
    cat.setUseVisualizer(true);

    // To aid with visualization.
    cat.setGravity(Vec3(0, 0, 0));

    // --- Prepare for integration.
    State& initState = cat.initSystem();

    // Configure the visualization.
    cat.getMultibodySystem().realize(initState, Stage::Position);

    ModelVisualizer & modelVis = cat.updVisualizer();
    Visualizer & vis = modelVis.updSimbodyVisualizer();
    vis.setBackgroundType(Visualizer::GroundAndSky);
    vis.setShowShadows(true);
    vis.setWindowTitle("FelisCatus");
    Visualizer::InputSilo & silo = modelVis.updInputSilo();

    modelVis.show(initState);

    CoordinateSet& coordinates = cat.updCoordinateSet();
    coordinates[0].setValue(initState, 0);

    // Construct an integrator.
    SimTK::RungeKuttaMersonIntegrator integrator(cat.getMultibodySystem());
    integrator.setAccuracy(1.0e-6);

    // Construct a manager to run the integration.
    Manager manager(cat, integrator);

    // Print out details of the model
    cat.printDetailedInfo(initState, cout);

    // Integrate from initial time to final time
    manager.setInitialTime(0);
    manager.setFinalTime(1.0);

    sleep(1.0);

    manager.integrate(initState);

    return EXIT_SUCCESS;
}
