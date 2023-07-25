#include "Settings.h"

#include <omp.h>

stark::Settings::Settings()
{
	// Initialize default parameters
	this->execution.n_threads = omp_get_num_threads();
	
	//// Adaptive Time step
	this->simulation.adaptive_time_step.value = 0.01; // [s]
	this->simulation.adaptive_time_step.min = 0.0; // [s]
	this->simulation.adaptive_time_step.max = 0.01; // [s]
	this->simulation.adaptive_time_step.success_multiplier = 1.2;
	this->simulation.adaptive_time_step.failure_multiplier = 0.5;
	this->simulation.adaptive_time_step.n_successful_iterations_to_increase = 5;

	//// Adaptive Contact stiffness
	this->simulation.adaptive_time_step.value = 1e3;
	this->simulation.adaptive_time_step.min = 1e3;
	this->simulation.adaptive_time_step.max = 1e15;
	this->simulation.adaptive_time_step.success_multiplier = 1.0;
	this->simulation.adaptive_time_step.failure_multiplier = 2.0;
	this->simulation.adaptive_time_step.n_successful_iterations_to_increase = std::numeric_limits<int>::max();
}