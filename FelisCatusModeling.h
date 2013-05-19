
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
using OpenSim::DisplayGeometry;
using OpenSim::LinearFunction;
using OpenSim::PinJoint;
using OpenSim::BallJoint;
using OpenSim::WeldJoint;
using OpenSim::CustomJoint;
using OpenSim::Model;
using OpenSim::SpatialTransform;
using OpenSim::CoordinateActuator;

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
 * the model contains "dummy" frames (i.e., massless bodies) that can be used
 * to specify rotational relations between the anterior and posterior segments
 * of the cat.
 * */

class FelisCatusModeling
{
public:

    // @param modelName Name of the OpenSim::Model object being created.
    // @param fileName Name/path of the OpenSim::Model file to write.
    void makeModel(string modelName, string fileName)
    {
        // Create the model.
        cat = Model();
        cat.setName(modelName);
        // TODO cat.setGravity(Vec3(0, -9.81, 0));
        cat.setGravity(Vec3(0, 0, 0));

        // Constants/parameters.
        segmentalLength = 0.2;			// m
        segmentalDiam = 0.15;			// m
        segmentalMass = 1;				// kg
		legsLength = 0.12;				// m
		legsDiam = 0.1*legsLength;		// m
		legsWidth = 0.6* segmentalDiam; // m; sum of both legs (60% distance across the belly)
		legsMass = 0.2;					// kg
        // See Kane and Scher (1969) about next 3 lines:
        I = 1; // kg-m^2; transverse moment of inertia
        JIratio = 0.25;
        double J = JIratio * I; // axial moment of inertia
        double Ixx = J; // + mass*(bodyLength/2.0)*(bodyLength/2.0); // kg-m^2, shifted because of displaced joint TODO might be unneccessary? I is about COM.
        double Iyy = I; // kg-m^2
        double Izz = I; // kg-m^2
        double Ixy = 0; // kg-m^2
        double Ixz = 0; // kg-m^2
        double Iyz = 0; // kg-m^2
        segmentalInertia = Inertia(Ixx, Iyy, Izz, Ixz, Ixz, Iyz);
		legsInertia = Inertia(I, I, I, 0, 0, 0); // TODO
        zeroInertia = Inertia(0, 0, 0, 0, 0, 0);

        // Call the 'add' methods below (NOTE: the order of these calls matters, in general)
        addBodies();
        addDisplayGeometry();
        addJoints();
		addContactGeometry();
		addContactForces();
        addCoordinateLimitForces();
        addActuators();

        cat.print(fileName);
    }

protected:

    // Member variables.
    Model cat;
    Body * anteriorBody;
    Body * posteriorBody;
	Body * anteriorLegs;
	Body * posteriorLegs;

    // Constants/parameters.
    double segmentalLength;
    double segmentalDiam;
    double segmentalMass;
	double legsLength;
    double legsDiam;
	double legsWidth;
    double legsMass;
    double I;
    double JIratio;
    Inertia segmentalInertia;
	Inertia legsInertia;
    Inertia zeroInertia;

    // Body construction and mass properties. This function does not include
    // massless bodies necessary to define a joint type.
    void addBodies()
    {
        // Anterior half of cat.
        anteriorBody = new Body();
        anteriorBody->setName("anteriorBody");
        anteriorBody->setMass(segmentalMass);
        anteriorBody->setMassCenter(Vec3(-0.5 * segmentalLength, 0, 0));
        anteriorBody->setInertia(segmentalInertia);

        // Posterior half of cat.
        posteriorBody = new Body();
        posteriorBody->setName("posteriorBody");
        posteriorBody->setMass(segmentalMass);
        posteriorBody->setMassCenter(Vec3(0.5 * segmentalLength, 0, 0));
        posteriorBody->setInertia(segmentalInertia);

		// Legs.
        anteriorLegs = new Body();
        anteriorLegs->setName("anteriorLegs");
        anteriorLegs->setMass(legsMass);
        anteriorLegs->setMassCenter(Vec3(0.5 * legsLength, 0, 0));
        anteriorLegs->setInertia(legsInertia);
		
		posteriorLegs = new Body();
        posteriorLegs->setName("posteriorLegs");
        posteriorLegs->setMass(legsMass);
        posteriorLegs->setMassCenter(Vec3(0.5 * legsLength, 0, 0));
        posteriorLegs->setInertia(legsInertia);
    }

     // This method is responsible for adding anteriorBody and posteriorBody to
     // the model.
    virtual void addJoints() = 0;

     // For help with creating massless Body's (frames). If we weren't simply
     // building a model, creating a pointer like this wouldn't be a good
     // idea.
    Body * newMasslessBody(string name)
    {
        Body * massless = new Body();
        massless->setName(name);
        massless->setMass(0.0);
        massless->setMassCenter(Vec3(0.0, 0.0, 0.0));
        massless->setInertia(zeroInertia);
        return massless;
    }

    // Adds contact geometry for when cat hits the ground.
	virtual void addContactGeometry()
	{
		// TODO - do we really want this?
	}

    // Adds contact forces to act when contact geometries come
	// into contact.
	virtual void addContactForces()
	{
		// TODO - do we really want this?
	}

    // Resist joint motions past their limits.
    virtual void addCoordinateLimitForces()
    {
        // TODO - do we really want this?
    }

    // Adds actuators to degrees of freedom allowed by joints.
	virtual void addActuators() {
		// TODO
		
		/*
		TorqueActuator * testActuator = new TorqueActuator("anteriorBody",
        "posteriorBody");
        testActuator->setTorqueIsGlobal(false);
        testActuator->setAxis(Vec3(1.0, 0.0, 0.0));
        double optimalForce = 250.0;
        testActuator->setOptimalForce(optimalForce);
		*/
	};
    
    // Adds display geometry to anteriorBody and posteriorBody.
    void addDisplayGeometry()
    {
        Body & ground = cat.getGroundBody();

        // Ground.
        ground.addDisplayGeometry("treadmill.vtp");

        // Anterior body.
        DisplayGeometry * anteriorDisplay = 
            new DisplayGeometry("sphere.vtp");
		//            "feliscatus_cylinder_with_two_offset_feet_nubs.obj");
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
            new DisplayGeometry("sphere.vtp");
		//            "feliscatus_cylinder_with_two_offset_feet_nubs.obj");
        posteriorDisplay->setOpacity(0.5);
        posteriorDisplay->setColor(Vec3(0.7, 0.7, 0.7));

        posteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(
                posteriorDisplay);
        posteriorBody->updDisplayer()->setShowAxes(true);
		posteriorBody->updDisplayer()->setScaleFactors(
                Vec3(segmentalLength, segmentalDiam, segmentalDiam));

		// Legs.
		DisplayGeometry * anteriorLegsDisplay = 
            new DisplayGeometry("box.vtp");
        anteriorLegsDisplay->setOpacity(0.5);
        anteriorLegsDisplay->setColor(Vec3(0.7, 0.7, 0.7));

        anteriorLegs->updDisplayer()->updGeometrySet().adoptAndAppend(
                anteriorLegsDisplay);
        anteriorLegs->updDisplayer()->setShowAxes(true);
		anteriorLegs->updDisplayer()->setScaleFactors(
                Vec3(legsLength, legsDiam, legsWidth));

		DisplayGeometry * posteriorLegsDisplay = 
            new DisplayGeometry("box.vtp");
        posteriorLegsDisplay->setOpacity(0.5);
        posteriorLegsDisplay->setColor(Vec3(0.7, 0.7, 0.7));

        posteriorLegs->updDisplayer()->updGeometrySet().adoptAndAppend(
                posteriorLegsDisplay);
        posteriorLegs->updDisplayer()->setShowAxes(true);
		posteriorLegs->updDisplayer()->setScaleFactors(
                Vec3(legsLength, legsDiam, legsWidth));
    }
};

#endif
