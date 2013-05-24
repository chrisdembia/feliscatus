
#include <iostream>

#include <OpenSim/OpenSim.h>

#include <FelisCatusModeling.h>
#include <FelisCatusOptimizerSystem.h>

using std::cout;
using std::endl;
using std::string;

using SimTK::Optimizer;

using OpenSim::FelisCatusOptimizerTool;

/**
 * Creates a FelisCatusOptimizerSystem and then optimizes it. Prints out an
 * osim file of the cat model (with controls) that results from the
 * optimization, as well as an XML file containing all the splines used for
 * controls.
 * */
int main(int argc, char * argv[])
{
    // Get the filename of the FelisCatusOptimizerTool serialization.
    // argc is the number of command line inputs, INCLUDING the name of the
    //      exectuable as well. Thus, it'll always be greater than/equal to 1.
    // argv is an array of space-delimited command line inputs, the first one
    //      necessarily being the name of the executable.
    string help = "Must specify the name of a FelisCatusOptimizerTool "
                  "serialization (setup/input file).\n\nExamples:\n\t"
                  "optimize runA_setup.xml\n";

    if (argc == 2)
    { // Correct number of inputs.

        // Parse inputs using the Tool class.
        string toolSetupFile = argv[1];
        FelisCatusOptimizerTool tool(toolSetupFile);
        string name = tool.get_results_directory();

        // Use inputs to create the optimizer system.
        FelisCatusOptimizerSystem sys(tool);

        // Create the optimizer with our system.
        SimTK::OptimizerAlgorithm algorithm;
        if (tool.get_optimizer_algorithm() == "InteriorPoint")
            algorithm = SimTK::InteriorPoint;
        else if (tool.get_optimizer_algorithm() == "LBFGS")
            algorithm = SimTK::LBFGS;
        else if (tool.get_optimizer_algorithm() == "LBFGSB")
            algorithm = SimTK::LBFGSB;
        else throw OpenSim::Exception("Unrecognized optimizer algorithm string.");

        Optimizer opt(sys, SimTK::InteriorPoint);
        // TODO choose tolerance better.
        opt.setConvergenceTolerance(0.01);
        opt.useNumericalGradient(true);
        opt.setMaxIterations(1000);
        opt.setLimitedMemoryHistory(500);

        // Initialize parameters for the optimization to be zero.
        Vector initParameters = sys.initialParameters();

        // And we're off!
        try
        {
            double f = opt.optimize(initParameters);

            // Print the optimized model so we can explore the resulting motion.
            sys.printModel(name + "_optimized.osim");

            // Print the control splines so we can explore the resulting actuation.
            sys.printPrescribedControllerFunctionSet(
                    name + "_optimized_parameters.xml");

            cout << "Done! f = " << f << endl;
        }
        catch (...)
        {
            // Print the last model so we have something to look at.
            sys.printModel(name + "_last.osim");
            sys.printPrescribedControllerFunctionSet(
                    name + "_last_parameters.xml");

            cout << "Exception thrown; optimization not achieved." << endl;

            // Don't want to give the appearance of normal operation.
            throw;
        }
    }
    else
    { // Too few/many inputs, etc.
        cout << "\nIncorrect input provided. " << help << endl;
        return 0;
    }

    return EXIT_SUCCESS;
};
