
#include <iostream>

#include <OpenSim/OpenSim.h>

#include <FelisCatusModeling.h>
#include <FelisCatusOptimizerSystem.h>

using std::cout;
using std::endl;
using std::string;

using SimTK::Optimizer;

/**
 * Creates a FelisCatusOptimizerSystem and then optimizes it.
 * */
int main(int argc, char * argv[])
{
    // Figure out from the user which cat model to use.
    // argc is the number of command line inputs, INCLUDING the name of the
    //      exectuable as well. Thus, it'll always be greater than/equal to 1.
    // argv is an array of space-delimited command line inputs, the first one
    //      necessarily being the name of the executable.
    string help = "User must provide the following inputs:\n\n\t"
                  "1: name of cat model file, WITHOUT .osim extension,\n\t"
                  "2: name of the run, which we append to the cat model name.\n"
                  "\nExample:\n\t"
                  "optimize feliscatus_flexTwistRetractLegs testRun\n";
    if (argc == 3)
    { // Correct number of inputs.
        string modelFileName = argv[1];
        string postName = argv[2];
        FelisCatusOptimizerSystem sys(modelFileName + "_" + postName,
                modelFileName + ".osim");
        // TODO allow input of number of spline points.

        // Create the optimizer with our system.
        Optimizer opt(sys, SimTK::InteriorPoint);
        // TODO choose tolerance better.
        opt.setConvergenceTolerance(1.0);
        opt.useNumericalGradient(true);
        opt.setMaxIterations(1000);
        opt.setLimitedMemoryHistory(500);

        // Initialize parameters for the optimization to be zero.
        Vector initParameters(sys.getNumParameters(), 0.0);

        // And we're off!
        double f = opt.optimize(initParameters);

        // Print a message in the optimization log.
        char message[200];
        sprintf(message,
                "Done in %i objective function calls, with optimum value %f.",
                sys.getObjectiveCalls(), f);  
        sys.printToLog(message);

        cout << "Done!" << endl;
    }
    else
    { // Too few/many inputs, etc.
        cout << "\nIncorrect input provided. " << help << endl;
        return 0;
    }

    return EXIT_SUCCESS;
};