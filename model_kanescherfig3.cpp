
#include "FelisCatusModeling.h"

/** Creates a FelisCatus model with a spinal joint defined by Kane and Scher,
 * with the topology based off Figure 3 of Kane and Scher (1969).
 * */
int main(int argc, char *argv[])
{
    FelisCatusModeling fcm = FelisCatusModeling(
            "Leland_kanescherfig3", 
            FelisCatusModeling::KaneScherFig3,
            "feliscatus_kanescherfig3.osim");

    return EXIT_SUCCESS;
}
