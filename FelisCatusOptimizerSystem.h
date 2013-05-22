#ifndef FELISCATUSOPTIMIZERSYSTEM_H
#define FELISCATUSOPTIMIZERSYSTEM_H

#include <math.h>
#include <stdio.h>
// For mkdir:
#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <time.h>
#include <fstream>

#include <OpenSim/OpenSim.h>

#include "FelisCatusModeling.h"

using std::cout;
using std::endl;
using std::ofstream;
using std::string;
using std::vector;

using OpenSim::Array;
using OpenSim::CoordinateSet;
using OpenSim::FunctionSet;
using OpenSim::Manager;
using OpenSim::Model;
using OpenSim::PrescribedController;
using OpenSim::Set;
using OpenSim::SimmSpline;

using SimTK::OptimizerSystem;
using SimTK::Pi;
using SimTK::Real;
using SimTK::Stage;
using SimTK::State;
using SimTK::Vector;

namespace OpenSim
{

// TODO input is which terms to use in objective function.
// TODO normalize time
// TODO normalize max/min control.
// TODO move manager code outside of the objective function loop if possible.
// TODO figure out how to space out control points over time.

/**
 * Manages inputs to an optimization of cat-flipping via OpenSim's
 * serialization/XML abilities. This is NOT an AbstractTool.
 * */
class FelisCatusOptimizerTool : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(FelisCatusOptimizerTool, Object);
public:
    OpenSim_DECLARE_PROPERTY(results_directory, string,
            "Directory in which to save optimization log and results.");
    OpenSim_DECLARE_PROPERTY(model_filename, string,
            "Specifies path to model file, WITH .osim extension.");
    OpenSim_DECLARE_PROPERTY(duration, double,
            "Duration of forward dynamics simulation (seconds).");
    OpenSim_DECLARE_PROPERTY(num_optim_spline_points, int,
            "Number of points being optimized in each spline function. "
            "Constant across all splines. If an initial_parameters_filename is "
            "provided, the functions specified in that file must have the "
            "correct number of points. We do not error-check for this.");
    OpenSim_DECLARE_PROPERTY(initial_parameters_filename, string,
            "File containing FunctionSet of SimmSpline's used to initialize "
            "optimization parameters. If not provided, initial parameters are "
            "all 0.0. The name of each function must be identical to that of "
            "the actuator it is for. x values are ignored. The time values "
            "that are actually used in the simulation are equally spaced from "
            "t = 0 to t = duration, and there should be as many points in each "
            "function as given by the num_optim_spline_points property. y "
            "values should be nondimensional and between -1 and 1. NOTE the "
            "output optimized splines are NOT NONDIMENSIONAL. Be careful; we "
            "do not do any error checking.")

    FelisCatusOptimizerTool() : Object()
    {
        setNull();
        constructProperties();
    }

    FelisCatusOptimizerTool(const string &aFileName, bool
            aUpdateFromXMLNode=true) : Object(aFileName, aUpdateFromXMLNode)
    {
        setNull();
        constructProperties();
        updateFromXMLDocument();
    }

    void setNull() { }

    void constructProperties()
    {
        constructProperty_results_directory("results");
        constructProperty_model_filename("");
        constructProperty_duration(1.0);
        constructProperty_num_optim_spline_points(5);
        constructProperty_initial_parameters_filename("");
    }

};

}

/**
 * Finds a control input/trajectory that achieves certain desired
 * features of a cat's flipping maneuver. The optimizer system can operate on
 * different cat models, so long as they possess the following:
 * TODO
 *      1. a 'roll' coordinate equal to pi radians when anterior legs down.
 *      2. a 'twist' coordinate equal to 0 when posterior legs down.
 *
 * The control of the system is performed via a PrescribedController that uses
 * a spline trajectory for all actuators.
 * TODO describe how the parameters are ordered.
 * */
class FelisCatusOptimizerSystem : public OptimizerSystem
{
public:
    /**
     * @param tool Contains all necessary input information.
     * */
    FelisCatusOptimizerSystem(OpenSim::FelisCatusOptimizerTool & tool) :
        _tool(tool),
        _objectiveCalls(0),
        _objectiveFcnValueBestYet(SimTK::Infinity)
    {
        // Parse inputs.
        _name = _tool.get_results_directory();
        _cat = Model(_tool.get_model_filename());
        _numOptimSplinePoints = _tool.get_num_optim_spline_points();

        // Create a directory for all the output files we'll create.
#if defined(_WIN32)
        _mkdir(_name.c_str());
#else
        mkdir(_name.c_str(), 0777);
#endif

        // Serialize what we just deserialized, in the results dir.
        // To keep everything in one nice organized place.
        _tool.print(_name + "/" + _name + "_setup.xml");

        // Create a log.
        _optLog.open((_name + "/" + _name + "_log.txt").c_str(), ofstream::out);
        _optLog << "Felis Catus optimization log." << endl;
        time_t rawtime; time(&rawtime);
        _optLog << ctime(&rawtime);
        _optLog << "Model file name: " << _tool.get_model_filename() << endl;

        // Compute the number of optimization parameters we'll have.
        _numActuators = _cat.getActuators().getSize();
        int numParameters = _numActuators * _numOptimSplinePoints;
        setNumParameters(numParameters);

        // Add a PrescribedController to the model.
        PrescribedController * flipController = new PrescribedController();
        flipController->setName("flip_controller");

        // Set the PrescribedController to control all actuators.
        flipController->setActuators(_cat.getActuators());

        // Add the PrescribedController to the model.
        _cat.addController(flipController);

        // Create SimmSpline's for each actuator.
        double indexToTimeInSeconds =
            _tool.get_duration() / (double)(_numOptimSplinePoints - 1);
        for (int i = 0; i < _numActuators; i++)
        {
            // Create a function for this actuator.
            _splines.push_back(new SimmSpline());

            // Name this spline with the name of the corresponding actuator.
            _splines[i]->setName(_cat.getActuators().get(i).getName());

            // Add the correct number of points to this spline.
            for (int j = 0; j < _numOptimSplinePoints; j++)
            {
                // Scale the time points appropriately.
                double thisTime = (double)j * indexToTimeInSeconds;
                _splines[i]->addPoint(thisTime, 0.0);
            }

            // Tell the controller about this function.
            flipController->prescribeControlForActuator(i, _splines[i]);
        }

        // Set (nondimensional) parameter limits.
        Vector lowerLimits(getNumParameters(), -1.0);
        Vector upperLimits(getNumParameters(), 1.0);
        setParameterLimits(lowerLimits, upperLimits);

        // Create a header row in the log.
        _optLog << "objective_calls " <<
            "objective_fcn_value " <<
            "objective_fcn_value_best_yet ";
        for (int iAct = 0; iAct < _numActuators; iAct++)
        {
            for (int iPts = 0; iPts < _numOptimSplinePoints; iPts++)
            {
                _optLog << _cat.getActuators().get(iAct).getName() << "_"
                    << iPts << " ";
            }
        }
        _optLog << endl;
    }

    /**
     * With knowledge from the FelisCatusOptimizerTool, provides what should be
     * used as the initial parameters. If the tool's
     * initial_parameters_filename is empty, then the initial parameters are
     * set to 0.0.
     * */
    Vector initialParameters()
    {
        Vector initParams(getNumParameters(), 0.0);

        if (_tool.get_initial_parameters_filename() != "")
        { // A file is specified.
            // Deserialize the specified XML file.
            FunctionSet initFcns(_tool.get_initial_parameters_filename());

            // Write a copy of the FunctionSet to the results directory.
            initFcns.print(_name + "/" + _name + "_initial_parameters.xml");

            Array<string> initNames;
            initFcns.getNames(initNames);

            // This loop is set up so that, hopefully, the initFcns set's
            // functions do not need to be ordered in the same way as the
            // optimization parameters are.
            for (int iFcn = 0; iFcn < initNames.getSize(); iFcn++)
            { // Loop through each init function by name.

                // Get index in the model of the actuator with this name.
                int iAct = _cat.getActuators().getIndex(initNames[iFcn]);

                // Get access to the spline's methods.
                SimmSpline * fcn =
                    dynamic_cast<SimmSpline *>(&initFcns.get(iAct));

                for (int iPts = 0; iPts < _numOptimSplinePoints; iPts++)
                {
                    // Find the right index in the optimization parameters.
                    int paramIndex = iAct * _numOptimSplinePoints + iPts;

                    // Finally, transfer y value from input to init parameters.
                    initParams[paramIndex] = fcn->getY(iPts);
                }
            }
        }
        return initParams;
    }

    ~FelisCatusOptimizerSystem()
    {
        _optLog.close();
    }

    int objectiveFunc(const Vector & parameters,
            bool new_parameters,
            Real & f) const
    {
        // Increment the number of calls to this function.
        _objectiveCalls++;

        // Unpack parameters into the model: update spline points.
        for (int iAct = 0; iAct < _numActuators; iAct++)
        {
            // Max/min for all spline points for the i-th actuator.
            double minControl = _cat.getActuators().get(iAct).getMinControl();
            double maxControl = _cat.getActuators().get(iAct).getMaxControl();

            // Visit each point in this spline.
            for (int iPts = 0; iPts < _numOptimSplinePoints; iPts++)
            {
                int paramIndex = iAct * _numOptimSplinePoints + iPts;

                // Dimensional control value.
                double dimControlValue;

                if (parameters[paramIndex] < 0)
                { // parameter and minControl are both neg; must negate again.
                    dimControlValue = -minControl * parameters[paramIndex];
                }
                else
                {
                    dimControlValue = maxControl * parameters[paramIndex];
                }

                // Finally, edit the spline with the info from parameters.
                _splines[iAct]->setY(iPts, dimControlValue);
            }
        }

        // --- Run a forward dynamics simulation.
        State& initState = _cat.initSystem();

        // Construct an integrator.
        SimTK::RungeKuttaMersonIntegrator integrator(_cat.getMultibodySystem());
        integrator.setAccuracy(1.0e-6);

        // Construct a manager to run the integration.
        Manager manager(_cat, integrator);

        // TODO don't know why.
        _cat.getMultibodySystem().realize(initState, Stage::Acceleration);

        // Integrate from initial time to final time
        manager.setInitialTime(0);
        manager.setFinalTime(_tool.get_duration());
        // TODO change this initial time; penalize for taking too long.

        // --------------------------------------------------------------------
        manager.integrate(initState);
        // --------------------------------------------------------------------

        // --- Construct the objective function, term by term.
        // Create a copy of the init state; we need a state consistent w/model.
        State aState = initState;
        _cat.getMultibodySystem().realize(aState, Stage::Acceleration);
        const CoordinateSet& coordinates = _cat.getCoordinateSet();
        double roll = coordinates.get("roll").getValue(aState);
        double twist = coordinates.get("twist").getValue(aState);
		double hunch = coordinates.get("hunch").getValue(aState);
		double pitch = coordinates.get("pitch").getValue(aState);
        double deviationFromLegsDown = pow(roll - Pi, 2) + pow(twist, 2);
		double deviationFromSymmetry = pow(hunch - 2 * pitch, 2);
        // ====================================================================
        f = deviationFromLegsDown + deviationFromSymmetry;
        // ====================================================================

        // Update the log.
        _objectiveFcnValueBestYet = std::min(f, _objectiveFcnValueBestYet);
        _optLog << _objectiveCalls << " " << f <<
            " " <<  _objectiveFcnValueBestYet;
        for (int i = 0; i < parameters.size(); i++)
        {
            _optLog << " " << parameters[i];
        }
        _optLog << endl;

        // Print out to the terminal/console every so often.
        if (_objectiveCalls % 100 == 0)
            cout << "Objective call # " << _objectiveCalls << endl;

        return 0;
    }

    /**
     * Allows whoever has an instance of this class to write something to
     * the log. For example, the optimum value of the objective function.
     * */
    void printToLog(string message)
    {
        _optLog << message << endl;
    }

    int getObjectiveCalls() const { return _objectiveCalls; }

    /// @brief Prints the current model to the given file name.
    /// The file will be located in the directory containing the log file
    /// for this optimization run.
    void printModel(string filename) const
    {
        _cat.print(_name + "/" + filename);
    }

    /// @brief Serializes the current set of functions used for the actuators.
    /// The file will be located in the directory containing the log file
    /// for this optimization run.
    void printPrescribedControllerFunctionSet(string filename) const
    {
        // Create the FunctionSet that we'll then serialize.
        FunctionSet fset;
        fset.setSize(_splines.size());
        for (int iFcn = 0; iFcn < _splines.size(); iFcn++)
        {
            fset.insert(iFcn, *_splines[iFcn]);
        }
        fset.print(_name + "/" + filename);
    }

private:

    /// See constructor.
    string _name;

    /// See constructor.
    OpenSim::FelisCatusOptimizerTool & _tool;

    /// The model containing the attributes described in this class'
    /// description.
    mutable Model _cat;

    /// See constructor.
    int _numOptimSplinePoints;

    /// Number of actuators in the model.
    int _numActuators;

    /// A vector of the spline functions used in the PrescribedController.
    vector<SimmSpline *> _splines;

    // 'mutable' lets us modify the member inside const member functions, such
    // as objectiveFunc() above.

    /// Counts the number of calls to objectiveFunc.
    mutable int _objectiveCalls;

    /// To record details of this run.
    mutable ofstream _optLog;

    /// The best (lowest) value of the objective function, for logging.
    mutable double _objectiveFcnValueBestYet;

};

#endif
