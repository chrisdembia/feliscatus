
#include "FelisCatusModeling.h"

/** Creates a FelisCatus model with a spinal joint defined by Kane and Scher,
 * with the topology based off Figure 2 of Kane and Scher (1969).
 * */
int main(int argc, char *argv[])
{
    FelisCatusModeling fcm = FelisCatusModeling(
            "Leland_kanescherfig2", 
            FelisCatusModeling::KaneScherFig2,
            "feliscatus_kanescherfig2.osim");

    return EXIT_SUCCESS;
}
