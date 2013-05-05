
#include "FelisCatusModeling.h"

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
 * These salient features are implemented starting with Figure 2 of their paper.
 * 
 * We use the general convention that body origins are at joint locations,
 * and thus mass centers are specified to be some distance away. Furthermore,
 * the model contains several "dummy" frames (i.e., massless
 * bodies) that are used to specify rotational relations between the anterior
 * and posterior segments of the cat.
 * */
class KaneScherFig2Modeling : public FelisCatusModeling
{
    void addJoints();
};


/** Creates a FelisCatus model with a spinal joint defined by Kane and Scher,
 * with the topology based off Figure 2 of Kane and Scher (1969).
 * */
int main(int argc, char *argv[])
{
    KaneScherFig2Modeling m;
    m.makeModel("Leland_kanescherfig2", "feliscatus_kanescherfig2.osim");

    return EXIT_SUCCESS;
}

void KaneScherFig2Modeling::addJoints()
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

    // -- Add bodies to model.
    // This must be done after the joints for these bodies are added.
    cat.addBody(anteriorBody);
    cat.addBody(posteriorBody);
}

