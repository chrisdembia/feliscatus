
#include "FelisCatusModeling.h"

/**
 * This model is informed by how, in OpenSim human models, the
 * pelvis is connected to the ground via a FreeJoint.
 * */
class DembiaSketchModeling : public FelisCatusModeling
{
    void addJoints();
};

/** Creates a FelisCatus model with a spinal joint defined by Dembia and
 * Sketch.
 * */
int main(int argc, char *argv[])
{
    DembiaSketchModeling m = DembiaSketchModeling();
    m.makeModel("Leland_dembiasketch", "feliscatus_dembiasketch.osim");

    return EXIT_SUCCESS;
};

void DembiaSketchModeling::addJoints()
{
    Body & ground = cat.getGroundBody();

    // --- Joint between the ground and the anterior body.

    // -- This joint is a CustomJoint, which requires a SpatialTransform.
    // This construction goes smoothly seemingly only if we define the
    // SpatialTransform first, and pass it to the CustomJoint as a
    // constructor argument.  This was gleaned by reading the source code,
    // particularly CustomJoint::constructCoordinates().
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

    // -- Add bodies to model.
    // This must be done after the joints for these bodies are added.
    cat.addBody(anteriorBody);
    cat.addBody(posteriorBody);
}
