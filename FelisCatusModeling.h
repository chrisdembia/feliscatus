
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
using OpenSim::Coordinate;
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
using OpenSim::CoordinateLimitForce;

using SimTK::Inertia;
using SimTK::Pi;
using SimTK::Rotation;
using SimTK::Transform;
using SimTK::Vec3;
using SimTK::convertDegreesToRadians;

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
        cat.setGravity(Vec3(0, -9.81, 0));

        // Constants/parameters.
        segmentalLength = 0.175;		 // m
        segmentalDiam = 0.15;			 // m
        segmentalMass = 1;				 // kg
		legsLength = 0.125;				 // m
		legsDiam = 0.1 * legsLength;	 // m
		legsWidth = 0.6 * segmentalDiam; // m; sum of both legs (60% distance across the belly)
		legsMass = 0.2;					 // kg
        I = 1; // kg-m^2; transverse moment of inertia
        JIratio = 0.25; // from Kane and Scher (1969)
        double J = JIratio * I; // axial moment of inertia
        double Ixx = J; // + mass*(bodyLength/2.0)*(bodyLength/2.0); // kg-m^2, shifted because of displaced joint TODO might be unneccessary? I is about COM.
        double Iyy = I; // kg-m^2
        double Izz = I; // kg-m^2
        double Ixy = 0; // kg-m^2
        double Ixz = 0; // kg-m^2
        double Iyz = 0; // kg-m^2
        segmentalInertia = Inertia(Ixx, Iyy, Izz, Ixz, Ixz, Iyz);
		legsInertia = (legsMass/segmentalMass) * segmentalInertia;
        zeroInertia = Inertia(0, 0, 0, 0, 0, 0);

        addBodies();
        addDisplayGeometry();
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
    // massless bodies necessary to define some joint types.
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

    // For help with creating joints that require intermediate frames
	// (i.e., massless bodies).
    Body * newMasslessBody(string name)
    {
        Body * massless = new Body();
        massless->setName(name);
        massless->setMass(0);
        massless->setMassCenter(Vec3(0));
        massless->setInertia(zeroInertia);
        return massless;
    }
    
    // Adds display geometry to all bodies.
    void addDisplayGeometry()
    {
        Body & ground = cat.getGroundBody();

        // Ground.
        ground.addDisplayGeometry("treadmill.vtp");

        // Anterior body.
        DisplayGeometry * anteriorDisplay = 
            new DisplayGeometry("cylinder.vtp");
        anteriorDisplay->setOpacity(0.5);
        anteriorDisplay->setColor(Vec3(0.5, 0.5, 0.5));
        Rotation rot;
        rot.setRotationFromAngleAboutZ(0.5 * Pi);
        anteriorDisplay->setTransform(Transform(rot, Vec3(-0.5 * segmentalLength, 0, 0)));
		anteriorDisplay->setScaleFactors(Vec3(segmentalDiam, segmentalLength, segmentalDiam));
        anteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(anteriorDisplay);
        anteriorBody->updDisplayer()->setShowAxes(true);

        // Posterior body.
        DisplayGeometry * posteriorDisplay = 
            new DisplayGeometry("cylinder.vtp");
        posteriorDisplay->setOpacity(0.5);
        posteriorDisplay->setColor(Vec3(0.7, 0.7, 0.7));
		posteriorDisplay->setTransform(Transform(rot, Vec3(0.5 * segmentalLength, 0, 0)));
		posteriorDisplay->setScaleFactors(Vec3(segmentalDiam, segmentalLength, segmentalDiam));
        posteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(posteriorDisplay);
        posteriorBody->updDisplayer()->setShowAxes(true);

		// Legs.
		DisplayGeometry * anteriorLegsDisplay = new DisplayGeometry("box.vtp");
        anteriorLegsDisplay->setOpacity(0.5);
        anteriorLegsDisplay->setColor(Vec3(0.7, 0.7, 0.7));
		anteriorLegsDisplay->setTransform(Transform(Vec3(0.3 * legsLength, 0, 0)));
		anteriorLegsDisplay->setScaleFactors(Vec3(legsLength, legsDiam, legsWidth));
        anteriorLegs->updDisplayer()->updGeometrySet().adoptAndAppend(anteriorLegsDisplay);
        anteriorLegs->updDisplayer()->setShowAxes(true);

		DisplayGeometry * posteriorLegsDisplay = new DisplayGeometry("box.vtp");
        posteriorLegsDisplay->setOpacity(0.5);
        posteriorLegsDisplay->setColor(Vec3(0.7, 0.7, 0.7));
		posteriorLegsDisplay->setTransform(Transform(Vec3(0.3 * legsLength, 0, 0)));
		posteriorLegsDisplay->setScaleFactors(Vec3(legsLength, legsDiam, legsWidth));
        posteriorLegs->updDisplayer()->updGeometrySet().adoptAndAppend(posteriorLegsDisplay);
        posteriorLegs->updDisplayer()->setShowAxes(true);
    }
};

#endif
