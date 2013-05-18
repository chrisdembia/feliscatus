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
     * @param modeling An instance of a subclass of FelisCatusModeling, so that
     *      we can make a model that we will subsequently control.
     * @param numOptimSplinePoints The number of points (optimization parameters)
     *      for each spline. The total number of optimization parameters will
     *      be numSplinePoints * (# of actuators in model). There could be
     *      additional spline points (perhaps the first one?) that are NOT
     *      optimized for.
     * */
    FelisCatusOptimizerSystem(string name,
            FelisCatusModeling & modeling,
            int numSplinePoints=5)
        : _name(name), _modeling(modeling),
        _numOptimSplinePoints(numOptimSplinePoints),
        _objectiveCalls(0)
    {
        // Create the model with an appropriate name.

        // Compute the number of optimization parameters we'll have.
        // TODO
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

        // --------------------------------------------------------------------
        f = TODO;
        // --------------------------------------------------------------------

        return 0;
    }

    /**
     * Allows whoever has an instance of this class to write something to
     * the log. For example, the optimum value of the objective function.
     * */
    void printToLog(string message)
    {
        log << message << endl;
    }

    int getObjectiveCalls() const { return _objectiveCalls; }

private:

    /// See constructor.
    string _name;

    /// See constructor.
    FelisCatusModeling _modeling;

    /// See constructor.
    int _numOptimSplinePoints;

    /// Counts the number of calls to objectiveFunc.
    mutable int _objectiveCalls;
};

#endif
