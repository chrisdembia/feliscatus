
#include <iostream>

#include <OpenSim/OpenSim.h>

using std::cout;
using std::endl;
using std::string;

using OpenSim::Array;
using OpenSim::Coordinate;
using OpenSim::CoordinateActuator;
using OpenSim::Model;
using OpenSim::Set;
using OpenSim::SimmSpline;
using OpenSim::Storage;

using SimTK::Function;

int main(int argc, char * argv[])
{
    // Get the names of the model (without .osim extension) and the file containing
	//		the model's prescribed motion trajectories (i.e., splines points for
	//		each model coordinate).
    // argc is the number of command line inputs, INCLUDING the name of the
    //      exectuable as well. Thus, it'll always be greater than/equal to 1.
    // argv is an array of space-delimited command line inputs, the first one
    //      necessarily being the name of the executable.
    string help = "Must specify the name of a FelisCatus model and a FunctionSet "
                  "file containing trajectories for each of the model's "
				  "coordinates. \n\nExamples:\n\t prescribe feliscatus_"
                  "TwistHunchWagRetractLegs TwistHunchWagRetractLegs_"
				  "trajectories.xml\n";

    if (argc == 3)
    { // Correct number of inputs.

        // Parse inputs.
        string modelFile = argv[1];
		string coordFile = argv[2];

		// Create and rename model.
        OpenSim::Model cat = OpenSim::Model(modelFile + ".osim");
		cat.setName(modelFile + "_prescribed");

		// Create a coordinate storage object from the input FunctionSet serialization.
        OpenSim::FunctionSet coordFcns = OpenSim::FunctionSet(coordFile);

		// Create a prescribed-motion spline for each coordinate from
		// the input-file data.
        OpenSim::Set<OpenSim::Actuator> acts = cat.getActuators();
		int numActuators = acts.getSize();

		for (int iAct = 0; iAct < numActuators; iAct++) {

            CoordinateActuator * currAct = dynamic_cast<CoordinateActuator *>(&acts.get(iAct));
            string coordName = currAct->get_coordinate();
            Coordinate & coord = cat.updCoordinateSet().get(coordName);

            coord.setDefaultIsPrescribed(1);
            coord.setPrescribedFunction(coordFcns.get(coordName));
		}
        cat.print(modelFile + "_prescribed.osim");
    }
    else
    { // Too few/many inputs, etc.
        cout << "\nIncorrect input provided. " << help << endl;
        return 0;
    }

    return EXIT_SUCCESS;
};
