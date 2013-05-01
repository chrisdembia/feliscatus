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
int main()
{
    // Open the model.
    Model cat("feliscatus.osim");
    cat.setUseVisualizer(true);

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
    manager.setFinalTime(5.0);

    // TODO manager.integrate(initState);

    // Memory management.

    return EXIT_SUCCESS;
}
