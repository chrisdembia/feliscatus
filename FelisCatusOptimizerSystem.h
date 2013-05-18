#ifndef FELISCATUSOPTIMIZERSYSTEM_H
#define FELISCATUSOPTIMIZERSYSTEM_H

#include <OpenSim/OpenSim.h>

#include "FelisCatusModeling.h"

using std::cout;
using std::endl;
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
            int numSplinePoints=5) :
        _name(name),
        _model(Model(modelFileName)),
        _numOptimSplinePoints(numOptimSplinePoints),
        _objectiveCalls(0)
    {
        // Create a log.
        _optLog = ofstream(name + ".txt", ofstream::out);
        // TODO ideally give date.
        _optLog << "Model file name: " << modelFileName << endl;

        // Compute the number of optimization parameters we'll have.
        int numActuators = _model.getActuators().getSize();
        int numParameters = numActuators * _numSplinePoints;
        setNumParameters(numParameters);

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

    }

    int objectiveFunc(const Vector & parameters,
            bool new_parameters,
            Real & f) const
    {
        // Increment the number of calls to this function.
        _objectiveCalls++;

        // --- Run a forward dynamics simulation.

        // Assign the new value of the objective function.
        // --------------------------------------------------------------------
        f = TODO;
        // --------------------------------------------------------------------

        // Update the log.


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

    /// Counts the number of calls to objectiveFunc.
    mutable int _objectiveCalls;

    /// To record details of this run.
    mutable ofstream _optLog;
};

#endif
