
#include "FelisCatusModeling.h"

/** Creates a FelisCatus model with a spinal joint defined by Dembia and
 * Sketch.
 * */
int main(int argc, char *argv[])
{
    FelisCatusModeling fcm = FelisCatusModeling(
            "Leland_dembiasketch", 
            FelisCatusModeling::DembiaSketch,
            "feliscatus_dembiasketch.osim");

    return EXIT_SUCCESS;
};
