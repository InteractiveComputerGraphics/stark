#include <iostream>

#include <Eigen/Dense>
#include "../stark/src/hola.h"

#include <stark>

int main()
{
	stark_hola();

	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "test";
	settings.output.output_directory = "../output/test";
	settings.output.codegen_directory = "../output/codegen";

	stark::models::Simulation simulation(settings);

	return 0;
}
