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

using OpenSim::Model;

using SimTK::OptimizerSystem;
using SimTK::Real;
using SimTK::Vector;

/**
 * Finds a control input/trajectory that achieves certain desired
 * features of a cat's flipping maneuver. The optimizer system can operate on
 * different cat models, so long as they possess the following:
 * TODO
 *
 * The control of the system is performed via a PrescribedController that uses
 * a spline trajectory for all actuators.
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
        _model(Model(modelFileName)),
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
        int numActuators = _model.getActuators().getSize();
        int numParameters = numActuators * _numOptimSplinePoints;
        // TODO setNumParameters(numParameters);
        setNumParameters(1);

        // Add a PrescribedController to the model.

        // Create SimmSpline's for each actuator.

        // Set parameter limits from the actuator torque limits in the model.
        // TODO sample code:
        /*
        Vector lowerLimits(MOS.getNumParameters(), 0.0);
        lowerLimits[1] = 10.0;

        // Can't turn off actuation after cycle ends.
        Vector upperLimits(MOS.getNumParameters(), 90.0);
        upperLimits[1] = 50.0;

        MOS.setParameterLimits(lowerLimits, upperLimits);
        */
        setParameterLimits(Vector(1, -5.0), Vector(1, 5.0));

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

        // Unpack parameters.

        // --- Run a forward dynamics simulation.

        // Assign the new value of the objective function.
        // --------------------------------------------------------------------
        f = pow(parameters[0] - 2.0, 2); // TODO;
        // --------------------------------------------------------------------

        // Update the log.
        _optLog << _objectiveCalls;
        for (int i = 0; i < parameters.size(); i++)
        {
            _optLog << " " << parameters[i];
        }
        _optLog << endl;


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
    Model _model;

    /// See constructor.
    int _numOptimSplinePoints;

    // 'mutable' lets us modify the member inside const member functions, such
    // as objectiveFunc() above.

    /// Counts the number of calls to objectiveFunc.
    mutable int _objectiveCalls;

    /// To record details of this run.
    mutable ofstream _optLog;
};

#endif
