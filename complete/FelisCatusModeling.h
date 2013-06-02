
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
using OpenSim::Coordinate;
using OpenSim::CoordinateSet;
using OpenSim::DisplayGeometry;
using OpenSim::Model;

using SimTK::Inertia;
using SimTK::Pi;
using SimTK::Rotation;
using SimTK::Transform;
using SimTK::Vec3;
using SimTK::State;
using SimTK::convertDegreesToRadians;

/**
 * Creates an OpenSim model of a cat, sufficient for exploring the cat's
 * righting reflex.
 *
 * We use the general convention that body origins are at joint locations,
 * and thus mass centers are specified to be some distance away. Furthermore,
 * the model generally contains "dummy" frames (i.e., massless bodies) that can
 * be used to specify rotational relations between the anterior and posterior
 * segments of the cat.
 *
 * We assume that all models are 2-segments models, with mass/inertial
 * properties given by the member variables of this class. The model can be
 * extended to have more than 2 Body's, but we assume that there are only 2
 * torso Body's/segments.
 * */
class FelisCatusModeling
{
public:

    FelisCatusModeling()
    {
        // Constants/parameters.
        _segmentalLength = 0.175;		         // m
        _segmentalDiam = 0.15;			         // m
        _segmentalMass = 1;				         // kg
        _segmentalTransverseMomentOfInertia = 1; // kg-m^2
        _JIratio = 0.25; // from Kane and Scher (1969)

        _zeroInertia = Inertia(0, 0, 0, 0, 0, 0);
    }

    // @param modelName Name of the OpenSim::Model object being created.
    virtual void makeModel(string modelName)
    {
        // Create the model.
        _cat = Model();
        _cat.setName(modelName);
        _cat.setGravity(Vec3(0, 0, 0));

        createBaseBodies();
        addBaseDisplayGeometry();
        addBaseJointsAndBodies();
        addBaseForcesAndActuation();
    }

    // @param fileName Name/path of the OpenSim::Model file to write.
    void printModel(string fileName)
    {
        _cat.print(fileName);
    }

protected:

    // Member variables.
    Model _cat;
    Body * _anteriorBody;
    Body * _posteriorBody;

    // Constants/parameters/properties.
    double _segmentalLength;
    double _segmentalDiam;
    double _segmentalMass;
    double _segmentalTransverseMomentOfInertia;
    double _JIratio;
    /// For convenience; not necessarily a cat property. May or may not be used.
    Inertia _zeroInertia;

    /// Body construction and mass properties. This function does not include
    /// massless bodies necessary to define some joint types.
    /// Does not add any Body's to the model.
    void createBaseBodies()
    {
        double segmentalAxialMoment = _JIratio * _segmentalTransverseMomentOfInertia;
        double Ixx = segmentalAxialMoment;
        double Iyy = _segmentalTransverseMomentOfInertia;
        double Izz = _segmentalTransverseMomentOfInertia;
        double Ixy = 0;
        double Ixz = 0;
        double Iyz = 0;
        Inertia segmentalInertia = Inertia(Ixx, Iyy, Izz, Ixz, Ixz, Iyz);

        // Anterior half of cat.
        _anteriorBody = new Body();
        _anteriorBody->setName("anteriorBody");
        _anteriorBody->setMass(_segmentalMass);
        _anteriorBody->setMassCenter(Vec3(-0.5 * _segmentalLength, 0, 0));
        _anteriorBody->setInertia(segmentalInertia);

        // Posterior half of cat.
        _posteriorBody = new Body();
        _posteriorBody->setName("posteriorBody");
        _posteriorBody->setMass(_segmentalMass);
        _posteriorBody->setMassCenter(Vec3(0.5 * _segmentalLength, 0, 0));
        _posteriorBody->setInertia(segmentalInertia);
    }

    /// Adds display geometry to all bodies.
    void addBaseDisplayGeometry()
    {
        Body & ground = _cat.getGroundBody();

        // Anterior body.
        DisplayGeometry * anteriorDisplay = 
            new DisplayGeometry("cylinder.vtp");
        anteriorDisplay->setOpacity(0.5);
        anteriorDisplay->setColor(Vec3(0.5, 0.5, 0.5));
        Rotation rot;
        rot.setRotationFromAngleAboutZ(0.5 * Pi);
        anteriorDisplay->setTransform(Transform(rot, Vec3(-0.5 * _segmentalLength, 0, 0)));
		anteriorDisplay->setScaleFactors(Vec3(_segmentalDiam, _segmentalLength, _segmentalDiam));
        _anteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(anteriorDisplay);
        _anteriorBody->updDisplayer()->setShowAxes(true);

        // Posterior body.
        DisplayGeometry * posteriorDisplay = 
            new DisplayGeometry("cylinder.vtp");
        posteriorDisplay->setOpacity(0.5);
        posteriorDisplay->setColor(Vec3(0.7, 0.7, 0.7));
		posteriorDisplay->setTransform(Transform(rot, Vec3(0.5 * _segmentalLength, 0, 0)));
		posteriorDisplay->setScaleFactors(Vec3(_segmentalDiam, _segmentalLength, _segmentalDiam));
        _posteriorBody->updDisplayer()->updGeometrySet().adoptAndAppend(posteriorDisplay);
        _posteriorBody->updDisplayer()->setShowAxes(true);
    }

    /** This method is repsonsible for adding anteriorBody and posteriorBody to
     * the model. */
    virtual void addBaseJointsAndBodies() = 0;

    virtual void addBaseForcesAndActuation() { }

    /// For help with creating joints that require intermediate frames
    /// (i.e., massless bodies).
    Body * newMasslessBody(string name)
    {
        Body * massless = new Body();
        massless->setName(name);
        massless->setMass(0);
        massless->setMassCenter(Vec3(0));
        massless->setInertia(_zeroInertia);
        return massless;
    }
};

#endif
