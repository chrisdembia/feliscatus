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
using OpenSim::CoordinateSet;
using OpenSim::CustomJoint;
using OpenSim::DisplayGeometry;
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
 * This model allows for the "salient features of motion" of a falling cat
 * first identified by Kane and Scher (1969). These include:
 * 	1) The torso bends but does not twist.
 * 	2) At release, the spine is bent forward. Afterwards, the spine is
 * 	   first bent to one side, the backward, then to the other side,
 * 	   and finally forwards again. At the end of this "oscillation,"
 * 	   the cat has turned over.
 * 	3) The backwards bend that the cat experiences during its turning
 * 	   maneuver is more pronounced that its initial or final forward
 * 	   bends.
 * We use the general convention that body origins are at joint locations,
 * and thus mass centers are specified to be some distance away. Furthermore,
 * the model contains several "dummy" frames (i.e., massless
 * bodies) that are used to specify rotational relations between the anterior
 * and posterior segments of the cat.
 * */
class FelisCatusModeling
{
public:

    /** The types of spinal joints that we are exploring for the model.
     * - DembiaSketch: informed by how, in OpenSim human models, the
     * pelvis is connected to the ground via a FreeJoint.
     *
     * - KaneScherFig2: from Kane and Scher (1969) with the topology based off
     *   Figure 2 of their paper (using frames N, A, and K).
     * - KaneScherFig3: from Kane and Scher (1969) with the topology based off
     *   Figure 3 of their paper.
     * */
    enum SpinalJointType
    {
        DembiaSketch,
        KaneScherFig2,
        KaneScherFig3
    };

    /**
     * @param modelName Name of the OpenSim::Model object being created.
     * @param fileName Name/path of the OpenSim::Model file to write.
     * */
    FelisCatusModeling(string modelName, SpinalJointType jointType, string
            fileName)
    {
        // Create the model.
        cat = Model();
        cat.setName(modelName);
        // TODO cat.setGravity(Vec3(0, -9.81, 0));

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
        // Only one of the Joint functions should ever be called.
        switch (jointType)
        {
            case DembiaSketch:
                addDembiaSketchJoint();
                break;
            case KaneScherFig2:
                addKaneScherFig2Joint();
                break;
            case KaneScherFig3:
                addKaneScherFig3Joint();
                break;
            default:
                cout << "SpinalJointType not supported." << endl;
                break;
        }
        addCoordinateLimitForces();
        addActuators();
        addControllers();

        // -- Add bodies to model.
        // This must be done after the joints for these bodies are added.
        cat.addBody(anteriorBody);
        cat.addBody(posteriorBody);

        cat.print(fileName);
    }

private:

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

    void addDembiaSketchJoint()
    {
        Body & ground = cat.getGroundBody();

        // --- Joint between the ground and the anterior body.

        // -- This joint is a CustomJoint, which requires a SpatialTransform.
        // This construction goes smoothly seemingly only if we define the
        // SpatialTransform first, and pass it to the CustomJoint as a constructor
        // argument.  This was gleaned by reading the source code, particularly
        // CustomJoint::constructCoordinates().
        // We are following the example of a CustomJoint given by gait2354.
        SpatialTransform groundAnteriorST;

        // Rotation, using the body-fixed/Euler ZXY convention.
        groundAnteriorST.updTransformAxis(0).setCoordinateNames(
                Array<string>("anterior_flip", 1));
        groundAnteriorST.updTransformAxis(0).setAxis(Vec3(0, 0, 1));

        groundAnteriorST.updTransformAxis(1).setCoordinateNames(
                Array<string>("anterior_sprawl", 1));
        groundAnteriorST.updTransformAxis(1).setAxis(Vec3(1, 0, 0));

        groundAnteriorST.updTransformAxis(2).setCoordinateNames(
                Array<string>("anterior_roll", 1));
        groundAnteriorST.updTransformAxis(2).setAxis(Vec3(0, 1, 0));

        // Translation.
        groundAnteriorST.updTransformAxis(3).setCoordinateNames(
                Array<string>("anterior_tx", 1));
        groundAnteriorST.updTransformAxis(3).setAxis(Vec3(1, 0, 0));
        groundAnteriorST.updTransformAxis(4).setCoordinateNames(
                Array<string>("anterior_ty", 1));
        groundAnteriorST.updTransformAxis(4).setAxis(Vec3(0, 1, 0));
        groundAnteriorST.updTransformAxis(5).setCoordinateNames(
                Array<string>("anterior_tz", 1));
        groundAnteriorST.updTransformAxis(5).setAxis(Vec3(0, 0, 1));

        // -- The actual joint definition.
        Vec3 locGAInGround(0);
        Vec3 orientGAInGround(0);

        // This joint is at the same location (in any frame) as the
        // anterior_posterior joint defined below.
        Vec3 locGAInAnterior(0);
        Vec3 orientGAInAnterior(0);
        CustomJoint * groundAnterior = new CustomJoint("ground_anterior",
                ground, locGAInGround, orientGAInGround,
                *anteriorBody, locGAInAnterior, orientGAInAnterior,
                groundAnteriorST);

        // Set properties of the joint's coordinates.
        CoordinateSet & groundAnteriorCS = groundAnterior->upd_CoordinateSet();

        // flip
        double groundAnteriorCS0range[2] = {-Pi, Pi};
        groundAnteriorCS[0].setRange(groundAnteriorCS0range);
        groundAnteriorCS[0].setDefaultValue(0.0);
        groundAnteriorCS[0].setDefaultLocked(false);

        // sprawl
        double groundAnteriorCS1range[2] = {-Pi, Pi};
        groundAnteriorCS[1].setRange(groundAnteriorCS1range);
        groundAnteriorCS[1].setDefaultValue(0.0);
        groundAnteriorCS[1].setDefaultLocked(false);

        // roll
        double groundAnteriorCS2range[2] = {-3 * Pi, 3 * Pi};
        groundAnteriorCS[2].setRange(groundAnteriorCS2range);
        groundAnteriorCS[2].setDefaultValue(0.0);
        groundAnteriorCS[2].setDefaultLocked(false);

        // tx
        double groundAnteriorCS3range[2] = {-10, 10};
        groundAnteriorCS[3].setRange(groundAnteriorCS3range);
        groundAnteriorCS[3].setDefaultValue(0.0);

        // ty
        double groundAnteriorCS4range[2] = {-1, 100};
        groundAnteriorCS[4].setRange(groundAnteriorCS4range);
        groundAnteriorCS[4].setDefaultValue(20);

        // tz
        double groundAnteriorCS5range[2] = {-5, 5};
        groundAnteriorCS[5].setRange(groundAnteriorCS5range);
        groundAnteriorCS[5].setDefaultValue(0.0);
        groundAnteriorCS[5].setDefaultLocked(false);

        // -- Joint between the anterior and posterior bodies.
        // TODO there is a u-joint class
        Vec3 locAPInAnterior(0);
        Vec3 orientAPInAnterior(0, 0, 0);
        Vec3 locAPInPosterior(0);
        Vec3 orientAPInPosterior(0);
        PinJoint * anteriorPosterior = new PinJoint("anterior_posterior",
                *anteriorBody, locAPInAnterior, orientAPInAnterior,
                *posteriorBody, locAPInPosterior, orientAPInPosterior);
        // Joint coordinates.
        CoordinateSet & anteriorPosteriorCS =
            anteriorPosterior->upd_CoordinateSet();
        anteriorPosteriorCS[0].setName("waist_hunch");
        double anteriorPosteriorCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
        anteriorPosteriorCS[0].setRange(anteriorPosteriorCS0range);
        anteriorPosteriorCS[0].setDefaultValue(0.0);
        anteriorPosteriorCS[0].setDefaultLocked(false);
    }

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

    /**
     * TODO
     * */
    void addKaneScherFig2Joint()
    {
        // MASSLESS bodies (i.e., defining coordinate frames for rotation). We
        // have attempted to be as faithful to Kane and Scher (1969) as
        // possible. See their paper to understand the notation used.

        // Frame in which the X-axis points along ray K.  K lies in the X-Y
        // (A1-A2) plane of the coordinate frame attached to the anterior half
        // of the cat.
        Body * K = newMasslessBody("K");

        // Frame with the following axes:
        //		X: X-axis of coordinate frame attached to the posterior body.
        //		Y: normal to X and lying in the plane defined by X and ray K
        //		Z: mutually perpendicular to X and Y
        // The posteriorBody is then rotated about frame P's X axis.
        // [CD] The last page of the paper I think identifies this frame as P.
        Body * P = newMasslessBody("P");

        // Frame in which X-axis points along ray N. N is mutually
        // perpendicular to the X-axes of the coordinate frames attached to the
        // anterior and posterior halves of the cat (A1 and B1, respectively).
        Body * N = newMasslessBody("N");
    }

    /**
     * The topology with this joint is:
     *              A
     *            /
     * ground -> Q 
     *            \
     *              B
     * */
    void addKaneScherFig3Joint()
    {
        Body & ground = cat.getGroundBody();

        // I'm calling massless bodies "frames". Here are all the ones I'm
        // using:
        // The frame containing A1 and B1.
        Body * frameQ = newMasslessBody("Q");
        // Frame rotated from Q about N by -gamma/2, to which anteriorBody is
        // attached.
        Body * Aintermed = newMasslessBody("Aintermed");
        // Frame rotated from Q about N by gamma/2, to which posteriorBody is
        // attached.
        Body * Bintermed = newMasslessBody("Bintermed");

        // -- Connect Q to ground.
        Vec3 locGQInGround(0);
        // TODO need some description here.
        // Y rotation to place the joint axis in the ground's X-Y plane.
        Vec3 orientGQInGround(0, 0.5 * Pi, 0);
        Vec3 locGQInFrameQ(0);
        // Undo previous rotation on other side of joint so that cat lies in
        // X-Y plane.
        Vec3 orientGQInFrameQ(0, -0.5 * Pi, 0);
        PinJoint * groundFrameQ = new PinJoint("ground_frameQ",
                ground, locGQInGround, orientGQInGround,
                *frameQ, locGQInFrameQ, orientGQInFrameQ);
        CoordinateSet & groundFrameQCS = groundFrameQ->upd_CoordinateSet();
        groundFrameQCS[0].setName("kane_psi");
        double groundFrameQCS0range[2] = {-Pi, Pi};
        groundFrameQCS[0].setRange(groundFrameQCS0range);
        groundFrameQCS[0].setDefaultValue(0.0);
        groundFrameQCS[0].setDefaultLocked(false);
        // TODO Change the above to a CustomJoint soon.

        // Connect Aintermed(iate) to Q.
        Vec3 locQAIinFrameQ(0);
        Vec3 orientQAIinFrameQ(0, 0, 0);
        Vec3 locQAIinAintermed(0);
        Vec3 orientQAIinAintermed(0, 0, 0);
        PinJoint * frameQAintermed = new PinJoint("frameQ_Aintermed",
                *frameQ, locQAIinFrameQ, orientQAIinFrameQ,
                *Aintermed, locQAIinAintermed, orientQAIinAintermed);
        CoordinateSet & frameQAintermedCS =
            frameQAintermed->upd_CoordinateSet();
        frameQAintermedCS[0].setName("kane_gammaA");
        double frameQAintermedCS0range[2] = {-0.5 * Pi, 0};
        frameQAintermedCS[0].setRange(frameQAintermedCS0range);
        frameQAintermedCS[0].setDefaultValue(0.0);
        frameQAintermedCS[0].setDefaultLocked(false);

        // Connect Bintermed(iate) to Q.
        Vec3 locQBIinFrameQ(0);
        Vec3 orientQBIinFrameQ(0, 0, 0);
        Vec3 locQBIinBintermed(0);
        Vec3 orientQBIinBintermed(0, 0, 0);
        PinJoint * frameQBintermed = new PinJoint("frameQ_Bintermed",
                *frameQ, locQBIinFrameQ, orientQBIinFrameQ,
                *Bintermed, locQBIinBintermed, orientQBIinBintermed);
        CoordinateSet & frameQBintermedCS =
            frameQBintermed->upd_CoordinateSet();
        frameQBintermedCS[0].setName("kane_gammaB");
        double frameQBintermedCS0range[2] = {0, 0.5 * Pi};
        frameQBintermedCS[0].setRange(frameQBintermedCS0range);
        frameQBintermedCS[0].setDefaultValue(0.0);
        frameQBintermedCS[0].setDefaultLocked(false);

        // Connect Aintermed to anteriorBody.
        Vec3 locAIAinAintermed(0);
        Vec3 orientAIAinAintermed(0, 0.5 * Pi, 0);
        Vec3 locAIAinAnterior(0);
        Vec3 orientAIAinAnterior(0, -0.5 * Pi, 0);
        PinJoint * AintermedAnterior = new PinJoint("Aintermed_anterior",
                *Aintermed, locAIAinAintermed, orientAIAinAintermed,
                *anteriorBody, locAIAinAnterior, orientAIAinAnterior);
        CoordinateSet & AintermedAnteriorCS =
            AintermedAnterior->upd_CoordinateSet();
        AintermedAnteriorCS[0].setName("kane_u_integ");
        double AintermedAnteriorCS0range[2] = {-2 * Pi, 2 * Pi};
        AintermedAnteriorCS[0].setRange(AintermedAnteriorCS0range);
        AintermedAnteriorCS[0].setDefaultValue(0.0);
        AintermedAnteriorCS[0].setDefaultLocked(false);

        // Connect Bintermed to posteriorBody.
        Vec3 locBIBinBintermed(0);
        Vec3 orientBIBinBintermed(0, 0.5 * Pi, 0);
        Vec3 locBIBinPosterior(0);
        Vec3 orientBIBinPosterior(0, -0.5 * Pi, 0);
        PinJoint * BintermedPosterior = new PinJoint("Bintermed_posterior",
                *Bintermed, locBIBinBintermed, orientBIBinBintermed,
                *posteriorBody, locBIBinPosterior, orientBIBinPosterior);
        CoordinateSet & BintermedPosteriorCS =
            BintermedPosterior->upd_CoordinateSet();
        BintermedPosteriorCS[0].setName("kane_v_integ");
        double BintermedPosteriorCS0range[2] = {-2 * Pi, 2 * Pi};
        BintermedPosteriorCS[0].setRange(BintermedPosteriorCS0range);
        BintermedPosteriorCS[0].setDefaultValue(0.0);
        BintermedPosteriorCS[0].setDefaultLocked(false);

        cat.addBody(frameQ);
        cat.addBody(Aintermed);
        cat.addBody(Bintermed);
        
        // TODO get rid of intermediates using GimbalJoint.
        // TODO constraint between intermediate frames.

    }

    void addCoordinateLimitForces()
    {
        // ---- Coordinate limits.
        // TODO 
    }

    void addActuators()
    {
        // ---- Actuators.
        /** TODO commenting out until joint is figured out.
          TorqueActuator * testActuator = new TorqueActuator("anteriorBody",
          "posteriorBody");
          testActuator->setTorqueIsGlobal(false);
          testActuator->setAxis(Vec3(1.0, 0.0, 0.0));
          double optimalForce = 250.0;
          testActuator->setOptimalForce(optimalForce);
          */
    }

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

    /** Adds display geometry to anteriorBody and posteriorBody.
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
