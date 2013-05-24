
#include <iostream>

#include <FelisCatusOptimizerSystem.h>

using std::cout;
using std::endl;
using std::string;

using OpenSim::FelisCatusOptimizerTool;

/**
 * Prints a template XML serialization of a FelisCatusOptimizerTool.
 * */
int main(int argc, char * argv[])
{
    string filename = "feliscatusoptimizertool_template.xml";
    FelisCatusOptimizerTool tool;
    tool.set_results_directory("results");
    tool.set_model_filename("");
    tool.set_duration(1.0);
    tool.set_optimizer_algorithm("InteriorPoint");
    tool.set_num_optim_spline_points(5);
    tool.set_anterior_legs_down_weight(1.0);
    tool.set_posterior_legs_down_weight(1.0);
    tool.set_sagittal_symmetry_weight(1.0);
    tool.set_legs_prepared_for_landing_weight(1.0);
    tool.set_initial_parameters_filename("");
    tool.print(filename);

    cout << "Printed template as " << filename << "." << endl;

    return EXIT_SUCCESS;
}
