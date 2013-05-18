
#include <iostream>

#include <OpenSim/OpenSim.h>

#include <FelisCatusModeling.h>
#include <FelisCatusOptimizerSystem.h>

using std::cout;
using std::endl;
using std::string;

/**
 * Creates a FelisCatusOptimizerSystem and then optimizes it.
 * */
int main(int argc, char * argv[])
{
    FelisCatusOptimizerSystem sys;
    // Figure out from the user which cat model to use.
    // argc is the number of command line inputs, INCLUDING the name of the
    //      exectuable as well. Thus, it'll always be greater than/equal to 1.
    // argv is an array of space-delimited command line inputs, the first one
    //      necessarily being the name of the executable.
    string help = "User must provide the following inputs:\n\n\t"
                  "1: name of cat modeling to use for this optimization,\n\t"
                  "2: name of the run, which we append to the cat model name.\n"
                  "\nExample:\n\t"
                  "optimize flexTwistRetractLegs testRun\n";
    if (argc == 3)
    {
        string preName = argv[2];
        if (strcmp(argv[1], "flexTwistRetractLegs") == 0)
        {
            sys = FelisCatusOptimizerSystem("flexTwistRetractLegs" + postName,
                    FlexTwistRetractLegsModeling());
        }
        else
        {
            cout << "\nUnrecognized cat modeling name. " << help << endl;
            return 0;
        }
    }
    else
    { // Too few/many inputs, etc.
        cout << "\nIncorrect input provided. " << help << endl;
        return 0;
    }

    // Create the optimizer with our system.
    Optimizer opt(sys, SimTK::InteriorPoint);
    // TODO choose tolerance better.
    opt.setConvergenceTolerance(1.0);
    opt.useNumericalGradient(true);
    opt.setMaxIterations(1000);
    opt.setLimitedMemoryHistory(500);

    // Initialize parameters for the optimization to be zero.
    Vector initParameters(MOS.getNumParameters(), 0.0);

    // And we're off!
    double f = opt.optimize(initParameters);

    // Print a message in the optimization log.
    char message[200];
    sprintf(message,
            "Done in %i objective function calls, with optimum value %f.",
            sys.getObjectiveCalls(), f);  
    sys.printToLog(message);

    return EXIT_SUCCESS;
};
