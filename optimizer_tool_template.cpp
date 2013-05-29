
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
    tool.set_model_filename("feliscatus_*FILL THIS IN*.osim");
    tool.set_duration(1.0);
    tool.set_optimizer_algorithm("BestAvailable");
    tool.set_num_optim_spline_points(20);
    tool.set_anterior_legs_down_weight(1.0);
    tool.set_posterior_legs_down_weight(1.0);
	tool.set_hunch_value(Pi/4);
	tool.set_hunch_weight(1.0);
	tool.set_sagittal_symmetry_weight(1.0);
	tool.set_wag_value(0.0);
	tool.set_wag_weight(1.0);
	tool.set_yaw_value(0.0);
	tool.set_yaw_weight(1.0);
	tool.set_legs_separation(Pi/2);
	tool.set_legs_separation_weight(1.0);
    tool.set_legs_prepared_for_landing_weight(1.0);
	tool.set_use_coordinate_limit_forces(true);
    tool.set_relative_velaccel_weight(0.1);
    tool.set_large_twist_penalty_weight(0.0);

    tool.set_taskspace_anterior_legs_down_weight(0.0);
    tool.set_taskspace_posterior_legs_down_weight(0.0);
    tool.set_desired_anterior_feet_pos_from_pivot_point_in_ground(Vec3(-1, -1, 0));
    tool.set_desired_posterior_feet_pos_from_pivot_point_in_ground(Vec3(1, -1, 0));
    
    tool.set_heavy_point_mass_legs(false);

    tool.set_initial_parameters_filename("feliscatusoptimizertool_initParams.xml");
    tool.print(filename);

    cout << "Printed template as " << filename << "." << endl;

    return EXIT_SUCCESS;
}
