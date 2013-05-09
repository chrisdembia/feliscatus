#ifndef FELISCATUSMODELING_H
#define FELISCATUSMODELING_H

#include <iostream>

#include <OpenSim/OpenSim.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using OpenSim::Array;
using OpenSim::Body;
using OpenSim::CoordinateCouplerConstraint;
using OpenSim::CoordinateSet;
using OpenSim::CustomJoint;
using OpenSim::DisplayGeometry;
using OpenSim::LinearFunction;
using OpenSim::PinJoint;
using OpenSim::Model;
using OpenSim::SpatialTransform;
using OpenSim::TorqueActuator;

using SimTK::Inertia;
using SimTK::Pi;
using SimTK::Rotation;
using SimTK::Transform;
using SimTK::Vec3;

/**
 * Creates an OpenSim model of a cat, sufficient for exploring the cat's
 * righting reflex.
 *
 * We use the general convention that body origins are at joint locations,
 * and thus mass centers are specified to be some distance away. Furthermore,
 * the model contains several "dummy" frames (i.e., massless
 * bodies) that are used to specify rotational relations between the anterior
 * and posterior segments of the cat.
 * */
class FelisCatusModeling
{
public:

    /**
     * @param modelName Name of the OpenSim::Model object being created.
     * @param fileName Name/path of the OpenSim::Model file to write.
     * */
    void makeModel(string modelName, string fileName)
    {
        // Create the model.
        cat = Model();
        cat.setName(modelName);
        // TODO cat.setGravity(Vec3(0, -9.81, 0));
        cat.setGravity(Vec3(0, 0, 0));

        // Constants/parameters.
        segmentalLength = 1; // m
        segmentalDiam = 1; // m
        segmentalMass = 1; // kg
        // See Kane and Scher (1969) about next 3 lines:
        I = 1; // kg-m^2; transverse moment of inertia.
        JIratio = 0.25;
        double J = JIratio * I; // axial moment of inertia;
        double Ixx = J; // + mass*(bodyLength/2.0)*(bodyLength/2.0); // kg-m^2, shifted because of displaced joint TODO might be unneccessary? I is about COM.
        double Iyy = I; // kg-m^2
        double Izz = I; // kg-m^2
        double Ixy = 0; // kg-m^2
        double Ixz = 0; // kg-m^2
        double Iyz = 0; // kg-m^2
        segmentalInertia = Inertia(Ixx, Iyy, Izz, Ixz, Ixz, Iyz);
        zeroInertia = Inertia(0, 0, 0, 0, 0, 0);

        // -- Call the 'add' methods below.
        // The order of these calls matters, in general.
        addBodies();
        addDisplayGeometry();
        addJoints();
        addCoordinateLimitForces();
        addActuators();
        addControllers();

        cat.print(fileName);
    }

protected:

    // ----- Member variables.
    Model cat;
    Body * anteriorBody;
    Body * posteriorBody;

    // -- Constants/parameters.
    double segmentalLength;
    double segmentalDiam;
    double segmentalMass;
    double I;
    double JIratio;
    Inertia segmentalInertia;
    Inertia zeroInertia;

    /** Body construction and mass properties. This function does not include
     * massless bodies necessary to define a joint type.
     * */
    void addBodies()
    {
        // Anterior half of cat.
        anteriorBody = new Body();
        anteriorBody->setName("anteriorBody");
        anteriorBody->setMass(segmentalMass);
        anteriorBody->setMassCenter(Vec3(-0.5 * segmentalLength, 0.0, 0.0));
        anteriorBody->setInertia(segmentalInertia);

        // Posterior half of cat.
        posteriorBody = new Body();
        posteriorBody->setName("posteriorBody");
        posteriorBody->setMass(segmentalMass);
        posteriorBody->setMassCenter(Vec3(0.5 * segmentalLength, 0.0, 0.0));
        posteriorBody->setInertia(segmentalInertia);

        // TODO change direction of mass center, and correspondingly, the g

        // NOTE: Why are we shifting the mass center at all? Don't we only want
        // to shift the body's moment of inertia (with parallel-axis theorem, see
        // above) and the location of the joint relative to the mass center?
    }

    /**
     * This method is responsible for adding anteriorBody and posteriorBody to
     * the model.
     * */
    virtual void addJoints() = 0;

    /**
     * For help with creating massless Body's (frames). If we weren't simply
     * building a model, creating a pointer like this wouldn't be a good
     * idea.
     * */
    Body * newMasslessBody(string name)
    {
        Body * massless = new Body();
        massless->setName(name);
        massless->setMass(0.0);
        massless->setMassCenter(Vec3(0.0, 0.0, 0.0));
        massless->setInertia(zeroInertia);
        return massless;
    }


    void addCoordinateLimitForces()
    {
        // ---- Coordinate limits.
        // TODO 
    }

    // ---- Actuators.
        /** TODO commenting out until joint is figured out.
          TorqueActuator * testActuator = new TorqueActuator("anteriorBody",
          "posteriorBody");
          testActuator->setTorqueIsGlobal(false);
          testActuator->setAxis(Vec3(1.0, 0.0, 0.0));
          double optimalForce = 250.0;
          testActuator->setOptimalForce(optimalForce);
    */
    virtual void addActuators() { };

    void addControllers()
    {
        // ---- Controllers.
        // "meat" is in computeControls()
        // pass actuators to controller in main()
        // get actuators in computeControls()
        // get coordinates to control from model & throw gains on top of the
        // errors
        // get desired acceleration
        // convert desired acceleration to force with mass (matrix)
    }
    
    /** 
     * Adds display geometry to anteriorBody and posteriorBody.
     * */
    void addDisplayGeometry()
    {
        Body & ground = cat.getGroundBody();

        // Ground.
        ground.addDisplayGeometry("treadmill.vtp");

        // Anterior body.
        DisplayGeometry * anteriorDisplay = 
            new DisplayGeometry(
                    "feliscatus_cylinder_with_two_offset_feet_nubs.obj");
        anteriorDisplay->setOpacity(0.5);
        anteriorDisplay->setColor(Vec3(0.5, 0.5, 0.5));
        Rotation rot;
        rot.setRotationFromAngleAboutY(Pi);
        anteriorDisplay->setTransform(Transform(rot));

        anteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(
                anteriorDisplay);
        anteriorBody->updDisplayer()->setShowAxes(true);
		anteriorBody->updDisplayer()->setScaleFactors(
                Vec3(segmentalLength, segmentalDiam, segmentalDiam));

        // Posterior body.
        DisplayGeometry * posteriorDisplay = 
            new DisplayGeometry(
                    "feliscatus_cylinder_with_two_offset_feet_nubs.obj");
        posteriorDisplay->setOpacity(0.5);
        posteriorDisplay->setColor(Vec3(0.7, 0.7, 0.7));

        posteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(
                posteriorDisplay);
        posteriorBody->updDisplayer()->setShowAxes(true);
		posteriorBody->updDisplayer()->setScaleFactors(
                Vec3(segmentalLength, segmentalDiam, segmentalDiam));
    }
};

#endif
