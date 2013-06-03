
#include <iostream>

#include <OpenSim/OpenSim.h>

#include "FlippinFelinesOptimizerSystem.h"

/**
 * Creates a FlippinFelinesOptimizerSystem and then optimizes it. Prints out an
 * osim file of the cat model (with controls) that results from the
 * optimization, as well as an XML file containing all the splines used for
 * controls.
 * */
int main(int argc, char * argv[])
{
    // argc is the number of command line inputs, INCLUDING the name of the
    //      exectuable as well. Thus, it'll always be greater than/equal to 1.
    // argv is an array of space-delimited command line inputs, the first one
    //      necessarily being the name of the executable.

    // Get the filename of the FlippinFelinesOptimizerTool serialization.
    if (argc == 2)
    { // Correct number of inputs.

        // Set-up
        // --------------------------------------------------------------------

        // Parse inputs using our Tool class.
        std::string toolSetupFile = argv[1];
        OpenSim::FlippinFelinesOptimizerTool tool(toolSetupFile);
        std::string name = tool.get_results_directory();

        // Use inputs to create the optimizer system.
        FlippinFelinesOptimizerSystem sys(tool);

        // Create the optimizer with our system.
        SimTK::Optimizer opt(sys, SimTK::BestAvailable);

        // Set optimizer settings.
        opt.setConvergenceTolerance(0.001);
        opt.useNumericalGradient(true);
        opt.setMaxIterations(100000);
        opt.setLimitedMemoryHistory(500);

        // Initialize parameters for the optimization as those determined
        // by our OptimizerSystem.
        SimTK::Vector initParameters = sys.initialParameters();

        // And we're off! Running the optimization
        // --------------------------------------------------------------------
        try
        {
            double f = opt.optimize(initParameters);

            // Print the optimized model so we can explore the resulting motion.
            sys.printModel(name + "_optimized.osim");

            // Print the control splines so we can explore the resulting actuation.
            sys.printPrescribedControllerFunctionSet(
                    name + "_optimized_parameters.xml");

            std::cout << "Done with " << name << "! f = " << f << std::endl;
        }
        catch (...)
        {
            // Print the last model/controls so we have something to look at.
            sys.printModel(name + "_last.osim");
            sys.printPrescribedControllerFunctionSet(
                    name + "_last_parameters.xml");

            std::cout << "Exception thrown; optimization not achieved." << std::endl;

            // Don't want to give the appearance of normal operation.
            throw;
        }
    }
    else
    { // Too few/many inputs, etc.
        std::cout << "\nIncorrect input provided. "
            "Must specify the name of a FlippinFelinesOptimizerTool "
            "serialization (setup/input file).\n\nExamples:\n\t"
            "optimize feliscatusoptimizertool_template.xml\n" << endl;
        return 0;
    }

    return EXIT_SUCCESS;
};
