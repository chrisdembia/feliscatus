
#include <OpenSim/OpenSim.h>
#include <FelisCatusOptimizerSystem.h>

using OpenSim::FelisCatusOptimizerTool;

int main(int argc, char *argv[])
{
    vector<bool> bothOptions;
    bothOptions.push_back(false);
    bothOptions.push_back(true);

    vector<bool> canTwist = bothOptions;
    vector<bool> canHunch = bothOptions;
    vector<bool> canWag = bothOptions;

    vector<string> whichLegs;
    whichLegs.push_back("");
    whichLegs.push_back("RigidLegs");
    whichLegs.push_back("RetractBackLeg");
    whichLegs.push_back("RetractLegs");

    vector<string> whichTail;
    whichTail.push_back("");
    whichTail.push_back("FixedPerch");
    whichTail.push_back("FreePerch");

    for (unsigned int it = 0; it < canTwist.size(); it++)
    {
        for (unsigned int ih = 0; ih < canHunch.size(); ih++)
        {
            for (unsigned int iw = 0; iw < canWag.size(); iw++)
            {
                for (int iL = 0; iL < whichLegs.size(); iL++)
                {
                    for (int iT = 0; iT < whichTail.size(); iT++)
                    {
                        string modifier = "_";
                        modifier += canTwist[it] ? "Twist" : "";
                        modifier += canHunch[ih] ? "Hunch" : "";
                        modifier += canWag[iw] ? "Wag" : "";

                        modifier += whichLegs[iL];
                        modifier += whichTail[iT];

                        string modelFileName = "feliscatus" + modifier + ".osim";
                        FelisCatusOptimizerTool tool;
                        tool.set_model_filename(modelFileName);
                        tool.set_results_directory(modifier);
                        tool.set_duration(1.0);
                        tool.set_optimizer_algorithm("BestAvailable");
                        tool.set_num_optim_spline_points(10);

                        tool.set_anterior_legs_down_weight(1.0);
                        if (canTwist[it] == true)
                            tool.set_posterior_legs_down_weight(1.0);
                        tool.set_sagittal_symmetry_weight(0.1);
                        tool.set_legs_prepared_for_landing_weight(0.01);

                        tool.set_use_coordinate_limit_forces(true);
                        tool.print("setup" + modifier + ".xml");
                    }
                }
            }
        }
    }
}
