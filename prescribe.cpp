
#include <iostream>

// For mkdir platform dependence:
#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <OpenSim/OpenSim.h>

#include "FelisCatusModeling.h"

using std::cout;
using std::endl;
using std::string;

using OpenSim::Array;
using OpenSim::Coordinate;
using OpenSim::CoordinateSet;
using OpenSim::Model;
using OpenSim::Set;
using OpenSim::SimmSpline;
using OpenSim::Storage;

using SimTK::Constraint;
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
    string help = "Must specify the name of a FelisCatus model and a "
                  "file containing trajectories for each of the model's "
				  "coordinates. \n\nExamples:\n\t prescribe feliscatus_"
                  "TwistHunchWagRetractLegs TwistHunchWagRetractLegs_"
				  "trajectories.sto\n";

    if (argc == 3)
    { // Correct number of inputs.

        // Parse inputs.
        string modelFile = argv[1];
		string coordFile = argv[2];

		// Create and rename model.
		Model cat = Model(modelFile + ".osim");
		cat.setName(modelFile + "_prescribed");

		// Create a coordinate storage object from the input .sto file
		// NOTE: .sto files can be created in the OpenSim GUI much like
		// ----  the excitation file for the tug-of-war in Lab 3.
		Storage coordinateSto = Storage(coordFile);
		
		// Get time from input file.
		Array<double> time;
		coordinateSto.getTimeColumn(time);

		// Create a prescribed-motion spline for each coordinate from
		// the input-file data.
		CoordinateSet coords = cat.getCoordinateSet();
		int numCoords = coords.getSize();

		vector<SimmSpline *> coordSplines;

		for (int i = 0; i < numCoords; i++) {
			// Get coordinate values (in rad) from input file, if not
			// locked. 
			Array<double> coordValue;
			Coordinate currCoord = coords.get(i);
			if (currCoord.getLocked( /*state*/ )) {
				coordinateSto.getDataColumn(currCoord.getName(), coordValue);
			}

			// Write time and coordinate values to spline for coordinate.

		}
    }
    else
    { // Too few/many inputs, etc.
        cout << "\nIncorrect input provided. " << help << endl;
        return 0;
    }

    return EXIT_SUCCESS;
};