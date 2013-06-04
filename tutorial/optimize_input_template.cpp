
#include <iostream>

#include "FlippinFelinesOptimizerSystem.h"

/**
 * Prints a template XML serialization of a FlippinFelinesOptimizerTool.
 * */
int main(int argc, char * argv[])
{
    std::string filename = "optimize_input_template.xml";
    OpenSim::FlippinFelinesOptimizerTool tool;
    tool.set_results_directory("results");
    tool.set_model_filename("flippinfelines_*FILL THIS IN*.osim");
    tool.set_num_optim_spline_points(20);
    tool.set_anterior_legs_down_weight(1.0);
    tool.set_posterior_legs_down_weight(1.0);
	tool.set_hunch_value(SimTK::Pi/4);
	tool.set_hunch_weight(1.0);
	tool.set_sagittal_symmetry_weight(1.0);
	tool.set_wag_value(0.0);
	tool.set_wag_weight(1.0);
	tool.set_yaw_value(0.0);
	tool.set_yaw_weight(1.0);
    tool.set_legs_prepared_for_landing_weight(1.0);
    tool.set_relative_velaccel_weight(0.1);

    tool.set_taskspace_anterior_legs_down_weight(0.0);
    tool.set_taskspace_posterior_legs_down_weight(0.0);
    tool.set_desired_anterior_feet_pos_from_pivot_point_in_ground(SimTK::Vec3(-1, -1, 0));
    tool.set_desired_posterior_feet_pos_from_pivot_point_in_ground(SimTK::Vec3(1, -1, 0));

	tool.set_use_coordinate_limit_forces(true);

    tool.set_initial_parameters_filename("initial_parameters.xml");
    tool.print(filename);

    std::cout << "Printed template as " << filename << "." << std::endl;

    return EXIT_SUCCESS;
}
