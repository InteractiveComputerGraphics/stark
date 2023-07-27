#include <iostream>

#include <Eigen/Dense>

#include <stark>

int main()
{
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "test";
	settings.output.output_directory = "../output/test";
	settings.output.codegen_directory = "../output/codegen";
	settings.execution.end_simulation_time = 0.1;

	stark::models::Simulation simulation(settings);
	simulation.stark.run();

	return 0;
}
