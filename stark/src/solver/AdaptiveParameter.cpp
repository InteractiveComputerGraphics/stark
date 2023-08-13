#include "AdaptiveParameter.h"

#include <cassert>
#include <algorithm>

void stark::AdaptiveParameter::set(double min, double value, double max)
{
	assert(min <= value && value <= max);
	this->min = min;
	this->value = value;
	this->max = max;
}

void stark::AdaptiveParameter::successful_iteration()
{
	this->current_successful_iterations++;
	if (this->current_successful_iterations == this->n_successful_iterations_to_increase) {
		this->current_successful_iterations = 0;
		this->value *= this->success_multiplier;
		this->value = std::max(this->min, std::min(this->value, this->max));
		this->current_successful_iterations = 0;
	}
}

void stark::AdaptiveParameter::failed_iteration()
{
	this->current_successful_iterations = 0;
	this->value *= this->failure_multiplier;
	this->value = std::max(this->min, std::min(this->value, this->max));
}
