#ifndef FELISCATUSOPTIMIZERSYSTEM_H
#define FELISCATUSOPTIMIZERSYSTEM_H

#include <stdio.h>
#include <time.h>
#include <fstream>

#include <OpenSim/OpenSim.h>

#include "FelisCatusModeling.h"

using std::cout;
using std::endl;
using std::ofstream;
using std::string;

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

// TODO move manager code outside of the objective function loop if possible.
// TODO figure out how to space out control points over time.

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
     * @param name Name of the run. We create a log file using this name.
     * @param modelFileName An OpenSim model filename of a model possessing the
     *      qualities described in this class' description.
     * @param numOptimSplinePoints The number of points (optimization parameters)
     *      for each spline. The total number of optimization parameters will
     *      be numSplinePoints * (# of actuators in model). There could be
     *      additional spline points (perhaps the first one?) that are NOT
     *      optimized for.
     * */
    FelisCatusOptimizerSystem(string name,
            string modelFileName,
            int numOptimSplinePoints=5) :
        _name(name),
        _cat(Model(modelFileName)),
        _numOptimSplinePoints(numOptimSplinePoints),
        _objectiveCalls(0)
    {
        // Create a log.
        _optLog.open((name + ".txt").c_str(), ofstream::out);
        _optLog << "Felis Catus optimization log." << endl;
        time_t rawtime; time(&rawtime);
        _optLog << ctime(&rawtime);
        _optLog << "Model file name: " << modelFileName << endl;

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
        _splines.setName("flip_control_fuctions");
        _splines.setSize(_numActuators);
        for (int i = 0; i < _numActuators; i++)
        {
            _splines.insert(i, SimmSpline());
            for (int j = 0; j < _numOptimSplinePoints; j++)
            { // TODO change times from index to something reasonable.
                _splines[i].addPoint((double)j, 0.0);
            }
            flipController->prescribeControlForActuator(i, &_splines[i]);
        }

        // Set parameter limits from the actuator torque limits in the model.
        Vector lowerLimits(getNumParameters(), 0.0);
        Vector upperLimits(getNumParameters(), 0.0);
        for (int iAct = 0; iAct < _numActuators; iAct++)
        {
            // Max/min for all spline points for the i-th actuator.
            double minControl = _cat.getActuators().get(iAct).getMinControl();
            double maxControl = _cat.getActuators().get(iAct).getMaxControl();
            for (int iPts = 0; iPts < _numOptimSplinePoints; iPts++)
            {
                int paramIndex = iAct * _numOptimSplinePoints + iPts;
                lowerLimits[paramIndex] = minControl;
                upperLimits[paramIndex] = maxControl;
            }
        }
        setParameterLimits(lowerLimits, upperLimits);
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
            for (int iPts = 0; iPts < _numOptimSplinePoints; iPts++)
            {
                int paramIndex = iAct * _numOptimSplinePoints + iPts;
                _splines[iAct].setY(iPts, parameters[paramIndex]);
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
        manager.setFinalTime(5.0);
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
        _optLog << _objectiveCalls;
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

private:

    /// See constructor.
    string _name;

    /// The model containing the attributes described in this class'
    //description.
    // TODO do we want this mutable?
    mutable Model _cat;

    /// See constructor.
    int _numOptimSplinePoints;

    /// Number of actuators in the model.
    int _numActuators;

    /// A set of the spline functions used in the PrescribedController.
    Set<SimmSpline> _splines;

    // 'mutable' lets us modify the member inside const member functions, such
    // as objectiveFunc() above.

    /// Counts the number of calls to objectiveFunc.
    mutable int _objectiveCalls;

    /// To record details of this run.
    mutable ofstream _optLog;
};

#endif
