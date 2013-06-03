#ifndef FELISCATUSOPTIMIZERSYSTEM_H
#define FELISCATUSOPTIMIZERSYSTEM_H

#include <math.h>
#include <stdio.h>
// For platform dependence of making a new directory:
#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <time.h>
#include <fstream>

#include <OpenSim/OpenSim.h>

using std::cout;
using std::endl;
using std::ofstream;
using std::string;
using std::vector;

using OpenSim::Array;
using OpenSim::CoordinateLimitForce;
using OpenSim::CoordinateSet;
using OpenSim::FunctionSet;
using OpenSim::Manager;
using OpenSim::Model;
using OpenSim::PrescribedController;
using OpenSim::Set;
using OpenSim::SimmSpline;
using OpenSim::Storage;

using SimTK::OptimizerSystem;
using SimTK::Pi;
using SimTK::Real;
using SimTK::Stage;
using SimTK::State;
using SimTK::Vector;
using SimTK::Vec3;

namespace OpenSim
{

/**
 * Manages inputs to an optimization of cat-flipping via OpenSim's
 * serialization/XML abilities. This is NOT an OpenSim::AbstractTool.
 * */
class FelisCatusOptimizerTool : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(FelisCatusOptimizerTool, Object);
public:

    // General properties.
    OpenSim_DECLARE_PROPERTY(results_directory, string,
            "Directory in which to save optimization log and results");
    OpenSim_DECLARE_PROPERTY(model_filename, string,
            "Specifies path to model file, WITH .osim extension");
    OpenSim_DECLARE_PROPERTY(num_optim_spline_points, int,
            "Number of points being optimized in each spline function. "
            "Constant across all splines. If an initial_parameters_filename is "
            "provided, the functions specified in that file must have the "
            "correct number of points. We do not error-check for this.");

    // Properties related to the objective function.
    OpenSim_DECLARE_PROPERTY(anterior_legs_down_weight, double,
            "Adds terms to the objective to minimize final value of "
            "(roll - Pi) and related speeds");
    OpenSim_DECLARE_PROPERTY(posterior_legs_down_weight, double,
            "Adds terms to the objective to minimize final value of "
            "(twist - 0) and related speeds");
	OpenSim_DECLARE_PROPERTY(hunch_value, double,
            "Final value for hunch coordinate (radians)");
	OpenSim_DECLARE_PROPERTY(hunch_weight, double,
            "Adds a term to the objective to minimize final difference "
            "between hunch and the specified hunch value, as well as the "
			"related speeds");
	OpenSim_DECLARE_PROPERTY(wag_value, double,
            "Final value for wag coordinate (radians)");
	OpenSim_DECLARE_PROPERTY(wag_weight, double,
            "Adds a term to the objective to minimize final difference "
            "between wag and the specified wag value, as well as the "
			"related speeds");
	OpenSim_DECLARE_PROPERTY(yaw_value, double,
            "Final value for yaw coordinate (radians)");
	OpenSim_DECLARE_PROPERTY(yaw_weight, double,
            "Adds a term to the objective to minimize final difference "
            "between yaw and the specified yaw value, as well as the "
			"related speeds");
	OpenSim_DECLARE_PROPERTY(sagittal_symmetry_weight, double,
            "Adds a term to the objective to minimize final value of "
            "(hunch + 2 * pitch)");
    OpenSim_DECLARE_PROPERTY(legs_prepared_for_landing_weight, double,
            "Adds terms to the objective to minimize final value of "
            "frontLegs, backLegs, and related speeds");
    OpenSim_DECLARE_PROPERTY(relative_velaccel_weight, double,
            "Weighting of velocity and acceleration objective terms, "
            "relative to the configuration term for a given objective weight "
            "above");

    OpenSim_DECLARE_PROPERTY(taskspace_anterior_legs_down_weight, double,
            "Weighting on deviation (as a squared vector magnitude) from "
            "desired_anterior_feet_pos_from_pivot_point_in_ground. This task-space "
            "objective would be used instead of anterior_legs_down.");
    OpenSim_DECLARE_PROPERTY(taskspace_posterior_legs_down_weight, double,
            "Weighting on deviation (as a squared vector magnitude) from "
            "desired_posterior_feet_pos_from_pivot_point_in_ground. This task-space "
            "objective would be used instead of anterior_legs_down and "
            "posterior_legs_down.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(
        desired_anterior_feet_pos_from_pivot_point_in_ground, Vec3,
            "Only relevant if using nonzero taskspace_anterior_legs_down_weight."); 
    OpenSim_DECLARE_OPTIONAL_PROPERTY(
        desired_posterior_feet_pos_from_pivot_point_in_ground, Vec3,
            "Only relevant if using nonzero taskspace_posterior_legs_down_weight."); 

    // Modifying the model before optimizing.
	OpenSim_DECLARE_PROPERTY(use_coordinate_limit_forces, bool,
            "TRUE: use coordinate limit forces, "
			"FALSE: ignore coordinate limit forces");

    // Setting initial parameters for the optimization.
    OpenSim_DECLARE_OPTIONAL_PROPERTY(initial_parameters_filename, string,
            "File containing FunctionSet of SimmSpline's used to initialize "
            "optimization parameters. If not provided, initial parameters are "
            "all 0.0, and this element must be DELETED from the XML file "
            "(cannot just leave it blank). The name of each function must be "
            "identical to that of the actuator it is for. x values are ignored. "
            "The time values that are actually used in the simulation are "
            "equally spaced from t = 0 to t = 1.0 s, and there should be "
			"as many points in each function as given by the num_optim_spline_points "
            "property. y values should be nondimensional and between -1 and 1 "
            "(negative values normalized by minControl if minControl is "
            "negative; otherwise. Otherwise the value is normalized by "
            "maxControl). NOTE the output optimized splines are NOT "
            "NONDIMENSIONAL. Be careful; " "we do not do any error checking.")

    FelisCatusOptimizerTool() : Object()
    {
        setNull();
        constructProperties();
    }

    /// This constructor allows for the de/serialization.
    FelisCatusOptimizerTool(const string &aFileName, bool
            aUpdateFromXMLNode=true) : Object(aFileName, aUpdateFromXMLNode)
    {
        setNull();
        constructProperties();
        // Must be called after constructProperties():
        updateFromXMLDocument();
    }

    void setNull() { }

    void constructProperties()
    {
        constructProperty_results_directory("results");
        constructProperty_model_filename("feliscatus_<FILL_THIS_IN>.osim");
        constructProperty_num_optim_spline_points(20);
        constructProperty_anterior_legs_down_weight(1.0);
        constructProperty_posterior_legs_down_weight(1.0);
		constructProperty_hunch_value(Pi/4);
		constructProperty_hunch_weight(1.0);
		constructProperty_wag_value(0.0);
		constructProperty_wag_weight(1.0);
		constructProperty_yaw_value(0.0);
		constructProperty_yaw_weight(1.0);
		constructProperty_sagittal_symmetry_weight(1.0);
        constructProperty_legs_prepared_for_landing_weight(1.0);
		constructProperty_use_coordinate_limit_forces(true);
        constructProperty_relative_velaccel_weight(1.0);

        constructProperty_taskspace_anterior_legs_down_weight(0.0);
        constructProperty_taskspace_posterior_legs_down_weight(0.0);
        constructProperty_desired_anterior_feet_pos_from_pivot_point_in_ground(Vec3(-1, -1, 0));
        constructProperty_desired_posterior_feet_pos_from_pivot_point_in_ground(Vec3(1, -1, 0));

        constructProperty_initial_parameters_filename("");
    }

};

}

/**
 * Finds a control input history that achieves certain desired
 * features of a cat's flipping maneuver, as determined by the objective
 * function. The control of the system is performed via a PrescribedController
 * that uses a spline for all actuators.
 *
 * Parameters are odered by actuator, then by spline point index for a given
 * actuator. Parameters are nondimensionalized by the min or max control
 * values for the associated actuator.
 * */
class FelisCatusOptimizerSystem : public OptimizerSystem
{
public:
    /**
     * @param tool Contains all input settings.
     * */
    FelisCatusOptimizerSystem(OpenSim::FelisCatusOptimizerTool & tool) :
        _tool(tool),
        _duration(1.0),
        _objectiveCalls(0),
        _objectiveFcnValueBestYet(SimTK::Infinity),
        _anteriorFeetPosFromPivotPointInAnterior(Vec3(-1, 1, 0)),
        _posteriorFeetPosFromPivotPointInPosterior(Vec3(1, 1, 0)),
        _relw(_tool.get_relative_velaccel_weight())
    {
        // Parse inputs
        // --------------------------------------------------------------------
        _name = _tool.get_results_directory();
        _cat = Model(_tool.get_model_filename());

		// -- Disable coordinate limit forces?
		if (!_tool.get_use_coordinate_limit_forces())
        {
            State& initState = _cat.initSystem();

            // Loop over all forces in model.
			for (int iFor = 0; iFor < _cat.getForceSet().getSize(); iFor++)
			{
				CoordinateLimitForce * LF = dynamic_cast<CoordinateLimitForce *> (&_cat.updForceSet().get(iFor));
				if (LF)
				{ // If it is a coordinate limit force, disable it.
					_cat.updForceSet().get(iFor).setDisabled(initState, true);
				}
			}
		}

        _numOptimSplinePoints = _tool.get_num_optim_spline_points();

        // Prepare output
        // --------------------------------------------------------------------

        // Create a directory for all the output files we'll create.
		#if defined(_WIN32)
			_mkdir(_name.c_str());
		#else
			mkdir(_name.c_str(), 0777);
		#endif

        // Serialize what we just deserialized, in the results dir,
        // to keep everything in one nice organized place.
        _tool.print(_name + "/" + _name + "_setup.xml");

        // Create a log for the objective function value and the separate terms
        // that go into it.
        _optLog.open((_name + "/" + _name + "_log.txt").c_str(), ofstream::out);
        _optLog << "Flippin Felines optimization log: " << name << endl;
        time_t rawtime; time(&rawtime);
        _optLog << ctime(&rawtime);
        _optLog << "Model file name: " << _tool.get_model_filename() << endl;

        // Compute the number of optimization parameters we'll have.
        _numActuators = _cat.getActuators().getSize();
        int numParameters = _numActuators * _numOptimSplinePoints;
        setNumParameters(numParameters);

        // Size the vector of best-yet splines.
        _splinesBestYet.resize(_numActuators);

        // Add a PrescribedController to the model.
        PrescribedController * flipController = new PrescribedController();
        flipController->setName("flip_controller");

        // Set the PrescribedController to control all actuators.
        flipController->setActuators(_cat.getActuators());

        // Add the PrescribedController to the model.
        _cat.addController(flipController);

        // - Create SimmSpline's for each actuator.
        double indexToTimeInSeconds =
            _duration / (double)(_numOptimSplinePoints - 1);
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
    }

    /**
     * With knowledge from the FelisCatusOptimizerTool, provides what should be
     * used as the initial parameters. If the tool's
     * initial_parameters_filename is empty, then all initial parameters are
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

            // This loop is set up so that the initFcns do not need to be
            // ordered in the same way as the
            // optimization parameters are. Also, the number of initFcns
			// specified by the initial parameters file can be greater than the
			// number of actuators in the model (assuming that the initial
			// parameters file AT LEAST contains the model's actuators).
            for (int iAct = 0; iAct < _numActuators; iAct++)
            { // Loop through the actuators in the model.

                // Get index of the initFcn corresponding to the actuator.
				int iFcn = initFcns.getIndex(_cat.getActuators().get(iAct).getName());

                // Get access to the spline's methods.
                SimmSpline * fcn =
                    dynamic_cast<SimmSpline *>(&initFcns.get(iFcn));

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

        // - Unpack parameters into the model: update spline points.
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

                if  (parameters[paramIndex] < 0 && minControl < 0)
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
        _cat.getMultibodySystem().realize(initState, Stage::Acceleration);

        // Integrate from initial time to final time
        manager.setInitialTime(0);
        manager.setFinalTime(_duration);

        // --------------------------------------------------------------------
        manager.integrate(initState);
        // --------------------------------------------------------------------

        // --- Construct the objective function value, term by term.
        // Will be writing to a log while constructing objective function val.
        if (_objectiveCalls % _logPeriod == 0)
            _optLog << _objectiveCalls << " ";

        // Create a copy of the init state; we need a consistent state.
        State aState = initState;
        _cat.getMultibodySystem().realize(aState, Stage::Acceleration);
        const CoordinateSet& coordinates = _cat.getCoordinateSet();
        double roll = coordinates.get("roll").getValue(aState);
        double rollRate = coordinates.get("roll").getSpeedValue(aState);
        double rollAccel = coordinates.get("roll").getAccelerationValue(aState);
        double twist = coordinates.get("twist").getValue(aState);
        double twistRate = coordinates.get("twist").getSpeedValue(aState);
        double twistAccel = coordinates.get("twist").getAccelerationValue(aState);
		double hunch = coordinates.get("hunch").getValue(aState);
		double hunchRate = coordinates.get("hunch").getSpeedValue(aState);
        double hunchAccel = coordinates.get("hunch").getAccelerationValue(aState);
		double pitch = coordinates.get("pitch").getValue(aState);
		double pitchRate = coordinates.get("pitch").getSpeedValue(aState);
        double pitchAccel = coordinates.get("pitch").getAccelerationValue(aState);
		double wag = coordinates.get("wag").getValue(aState);
		double wagRate = coordinates.get("wag").getSpeedValue(aState);
        double wagAccel = coordinates.get("wag").getAccelerationValue(aState);
		double yaw = coordinates.get("yaw").getValue(aState);
		double yawRate = coordinates.get("yaw").getSpeedValue(aState);
        double yawAccel = coordinates.get("yaw").getAccelerationValue(aState);

        // ====================================================================
        f = 0;
        f += anteriorLegsDownTerm(roll, rollRate, rollAccel);
        f += posteriorLegsDownTerm(twist, twistRate, twistAccel);
        f += hunchTerm(hunch, hunchRate, hunchAccel);
        f += wagTerm(wag, wagRate, wagAccel);
        f += yawTerm(yaw, yawRate, yawAccel);
        f += sagittalSymmetryTerm(hunch, pitch, pitchRate, pitchAccel);
        f += legsPreparedForLandingTerm(aState, coordinates);
        f += taskspaceTerms(aState, coordinates);
        // ====================================================================

        // Update the log.
        _lastCallWasBestYet = _thisCallIsBestYet;
        _thisCallIsBestYet = f <= _objectiveFcnValueBestYet;
        if (_thisCallIsBestYet) _objectiveFcnValueBestYet = f;

        if (_objectiveCalls % _logPeriod == 0)
            _optLog << " objfcn " << f << " objfcn_best_yet " << _objectiveFcnValueBestYet << endl;

        // If this is the best yet, save a copy of the splines.
        if (_thisCallIsBestYet)
        {
            for (unsigned int iFcn = 0; iFcn < _splinesBestYet.size(); iFcn++)
                _splinesBestYet[iFcn] = *_splines[iFcn];
        }

        // If we just got worse, print out the best-yet splines and
        // current model. Note: current model is NOT "best yet", but the
        // idea is that it'll be close. Also print out nondimensionalized
        // splines so that they can be used directly as input for a subsequent
        // optimization.
        if (_lastCallWasBestYet && !_thisCallIsBestYet)
        {
            printBestYetPrescribedControllerFunctionSet(
                    _name + "_best_yet_parameters.xml");
            printBestYetPrescribedControllerFunctionSet(
                    _name + "_best_yet_parameters_nomdim.xml", true);
            printModel(_name + "_best_yet.osim");
        }

        // Print out to the terminal/console every so often.
        if (_objectiveCalls % 100 == 0)
            cout << "Objective call # " << _objectiveCalls << endl;

        return 0;
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
        for (unsigned int iFcn = 0; iFcn < _splines.size(); iFcn++)
        {
            fset.insert(iFcn, *_splines[iFcn]);
        }
        fset.print(_name + "/" + filename);
    }

    /// @brief Serializes the set of functions associated with the best-yet
    /// value of the objective function.
    /// The file will be located in the directory containing the log file
    /// for this optimization run.
    /// @param nondimensionalize Divide spline values by minControl for the
    ///     actuator, if value is negative, and by maxControl, if value is
    ///     positive. If nondimensionalized, this FunctionSet can be used
    ///     as initial parameters.
    void printBestYetPrescribedControllerFunctionSet(string filename,
            bool nondimensionalize=false) const
    {
        // Create the FunctionSet that we'll then serialize.
        FunctionSet fset;
        fset.setSize(_splinesBestYet.size());
        for (unsigned int iFcn = 0; iFcn < _splinesBestYet.size(); iFcn++)
        {
            fset.insert(iFcn, _splinesBestYet[iFcn]);

            if (nondimensionalize)
            { // Go through each y-value and nondim. it based on its sign.

                // Max/min for all spline points for the i-th actuator.
                double minControl = _cat.getActuators().get(iFcn).getMinControl();
                double maxControl = _cat.getActuators().get(iFcn).getMaxControl();

                for (int iPts = 0; iPts < _numOptimSplinePoints; iPts++)
                {
                    SimmSpline * fcn = dynamic_cast<SimmSpline *>(&fset.get(iFcn));
                    double dimValue = fcn->getY(iPts);
                    double nonDimValue = 0;
                    if (dimValue < 0 && minControl < 0)
                    {
                        nonDimValue = -dimValue / minControl;
                    }
                    else
                    {
                        nonDimValue = dimValue / maxControl;
                    }
                    fcn->setY(iPts, nonDimValue);
                }
            }
        }
        fset.print(_name + "/" + filename);
    }

private:

    // Objective function terms
    // ========================================================================
    // These functions must be 'const' because objectiveFunc() is const; cannot
    // call member functions within a const member function unless those
    // functions are also const.

    // These functions are also defined inline (definition is inside the class
    // definition) for efficiency: the compiler will 'paste' the function body
    // into the place where the function is called.

    double anteriorLegsDownTerm(
            double roll, double rollRate, double rollAccel) const
    {
        if (_tool.get_anterior_legs_down_weight() != 0.0)
        {
            return processObjTerm("anterior_legs_down", _tool.get_anterior_legs_down_weight(),
                    pow(roll - Pi, 2) + _relw * pow(rollRate, 2) + _relw * pow(rollAccel, 2));
        }
        return 0.0;
    }

    double posteriorLegsDownTerm(
            double twist, double twistRate, double twistAccel) const
    {
        if (_tool.get_posterior_legs_down_weight() != 0.0)
        {
            return processObjTerm("posterior_legs_down", _tool.get_posterior_legs_down_weight(),
                    pow(twist - 0.0, 2) + _relw * pow(twistRate, 2) + _relw * pow(twistAccel, 2));
        }
        return 0.0;
    }

    double hunchTerm(
            double hunch, double hunchRate, double hunchAccel) const
    {
        if (_tool.get_hunch_weight() != 0.0)
        {
            double hunchGoal = _tool.get_hunch_value();
            return processObjTerm("hunch", _tool.get_hunch_weight(),
                    pow(hunch - hunchGoal, 2) + _relw * pow(hunchRate, 2) + _relw * pow(hunchAccel, 2));
        }
        return 0.0;
    }

    double wagTerm(
            double wag, double wagRate, double wagAccel) const
    {
        if (_tool.get_wag_weight() != 0.0)
        {
            double wagGoal = _tool.get_wag_value();
            return processObjTerm("wag", _tool.get_wag_weight(),
                    pow(wag - wagGoal, 2) + _relw * pow(wagRate, 2) + _relw * pow(wagAccel, 2));
        }
        return 0.0;
    }

    double yawTerm(
            double yaw, double yawRate, double yawAccel) const
    {
        if (_tool.get_yaw_weight() != 0.0)
        {
            double yawGoal = _tool.get_yaw_value();
            return processObjTerm("yaw", _tool.get_yaw_weight(),
                    pow(yaw - yawGoal, 2) + _relw * pow(yawRate, 2) + _relw * pow(yawAccel, 2));
        }
        return 0.0;
    }

    double sagittalSymmetryTerm(
            double hunch, double pitch, double pitchRate, double pitchAccel) const
    {
        if (_tool.get_sagittal_symmetry_weight() != 0.0)
        {
            return processObjTerm("sagittal_symmetry", _tool.get_sagittal_symmetry_weight(),
                    pow(hunch + 2 * pitch, 2) + _relw * pow(pitchRate, 2) + _relw * pow(pitchAccel, 2));
        }
        return 0.0;
    }

    double legsPreparedForLandingTerm(
            State& aState, const CoordinateSet & coordinates) const
    {
        if (_tool.get_legs_prepared_for_landing_weight() != 0.0)
        {
            // These values may not be available for all models.
            double frontLegs = coordinates.get("frontLegs").getValue(aState);
            double frontLegsRate = coordinates.get("frontLegs").getSpeedValue(aState);
            double frontLegsAccel = coordinates.get("frontLegs").getAccelerationValue(aState);
            double backLegs = coordinates.get("backLegs").getValue(aState);
            double backLegsRate = coordinates.get("backLegs").getSpeedValue(aState);
            double backLegsAccel = coordinates.get("backLegs").getAccelerationValue(aState);

            double termA = _tool.get_legs_prepared_for_landing_weight() * (
                    pow(frontLegs, 2) + _relw * pow(frontLegsRate, 2) + _relw * pow(frontLegsAccel, 2));
            double termB = _tool.get_legs_prepared_for_landing_weight() * (
                    pow(backLegs, 2) + _relw * pow(backLegsRate, 2) + _relw * pow(backLegsAccel, 2));
            return processObjTerm("legs_prepared_for_landing", 1.0, termA + termB);
        }
        return 0.0;
    }

    double taskspaceTerms(State& aState, const CoordinateSet & coordinates) const
    {
        if (_tool.get_taskspace_anterior_legs_down_weight() != 0.0 ||
                _tool.get_taskspace_posterior_legs_down_weight() != 0.0)
        { // Task-space flip conditions.

            double toReturn = 0.0;

            // Construct the position of the pivot point.
            double tx = coordinates.get("tx").getValue(aState);
            double ty = coordinates.get("ty").getValue(aState);
            double tz = coordinates.get("tz").getValue(aState);
            Vec3 pivotPointPosFromGroundPointInGround(tx, ty, tz);

            if (_tool.get_taskspace_anterior_legs_down_weight() != 0)
            {
                // All vectors instantiated in this scope are expressed
                // in ground frame.
                Vec3 anteriorFeetPosFromGroundPointInGround;
                _cat.getSimbodyEngine().transformPosition(aState, 
                        _cat.getBodySet().get("anteriorBody"),
                        _anteriorFeetPosFromPivotPointInAnterior,
                        _cat.getGroundBody(),
                        anteriorFeetPosFromGroundPointInGround);

                Vec3 anteriorFeetPosFromPivotPointInGround = 
                    anteriorFeetPosFromGroundPointInGround -
                    pivotPointPosFromGroundPointInGround;

                Vec3 diff = anteriorFeetPosFromPivotPointInGround -
                    _tool.get_desired_anterior_feet_pos_from_pivot_point_in_ground();

                // magnitude(diff)^2
                toReturn += processObjTerm("taskspace_anterior_legs_down",
                        _tool.get_taskspace_anterior_legs_down_weight(),
                        SimTK::dot(diff, diff));
            }

            if (_tool.get_taskspace_posterior_legs_down_weight() != 0.0)
            {
                // All vectors instantiated in this scope are expressed
                // in ground frame.
                Vec3 posteriorFeetPosFromGroundPointInGround;
                _cat.getSimbodyEngine().transformPosition(aState,
                        _cat.getBodySet().get("posteriorBody"),
                        _posteriorFeetPosFromPivotPointInPosterior,
                        _cat.getGroundBody(),
                        posteriorFeetPosFromGroundPointInGround);

                Vec3 posteriorFeetPosFromPivotPointInGround =
                    posteriorFeetPosFromGroundPointInGround -
                    pivotPointPosFromGroundPointInGround;

                Vec3 diff = posteriorFeetPosFromPivotPointInGround -
                    _tool.get_desired_posterior_feet_pos_from_pivot_point_in_ground();

                // magnitude(diff)^2
                toReturn += processObjTerm("taskspace_posterior_legs_down",
                        _tool.get_taskspace_posterior_legs_down_weight(),
                         SimTK::dot(diff, diff));
            }
            return toReturn;
        }
        return 0.0;
    }

    double processObjTerm(double name, double weight, double term) const
    {
        if (_objectiveCalls & _logPeriod == 0)
            _optLog << " " << name << " " << weight * term;
        return weight * term;
    }

    // Member variables
    // ========================================================================

    /// See constructor.
    string _name;

    /// See constructor.
    OpenSim::FelisCatusOptimizerTool & _tool;

    // 'mutable' lets us modify the member variable inside const member
    // functions, such as objectiveFunc() above.

    /// The model containing the attributes described in this class'
    /// description.
    mutable Model _cat;

    /// See constructor.
    int _numOptimSplinePoints;

    /// The amount of time for which the simulation runs. Units of seconds.
    double _duration;

    /// Number of actuators in the model.
    int _numActuators;

    /// A vector of the spline functions used in the PrescribedController.
    vector<SimmSpline *> _splines;

    /// Vector of the splines that gave the best objective value yet.
    mutable vector<SimmSpline> _splinesBestYet;

    /// Counts the number of calls to objectiveFunc.
    mutable int _objectiveCalls;

    /// To record details of this run.
    mutable ofstream _optLog;

    /// Period for how often objective terms are printed to the log.
    static const int _logPeriod = 100;

    /// The best (lowest) value of the objective function, for logging.
    mutable double _objectiveFcnValueBestYet;

    // To aid with conservative printing of best yet actuation.
    mutable bool _lastCallWasBestYet;
    mutable bool _thisCallIsBestYet;

    /// For task-space objectives.
    Vec3 _anteriorFeetPosFromPivotPointInAnterior;
    /// For task-space objectives.
    Vec3 _posteriorFeetPosFromPivotPointInPosterior;

    /// Relative weighting of velocity/acceleration terms.
    double _relw;

};

#endif
