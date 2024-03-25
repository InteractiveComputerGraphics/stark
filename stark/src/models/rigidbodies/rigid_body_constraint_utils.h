#pragma once
#include <string>
#include <memory>

#include <Eigen/Dense>
#include <fmt/format.h>

#include "../../core/Logger.h"


namespace stark
{
	double inf_norm(const Eigen::Vector3d& x)
	{
		return x.cwiseAbs().maxCoeff();
	}
	template<typename T, typename... Args>
	void log_parameters(std::shared_ptr<stark::core::Logger> logger, const std::string& constraint, const int idx, const std::string& constraint_name, const std::string& param_name, T&& value, Args&&... args) {
		logger.append_to_series(fmt::format("{} {:d} {} | {}", constraint, idx, constraint_name, param_name), fmt::format("{:.4e}", std::forward<T>(value)));
		log_parameters(logger, constraint, idx, constraint_name, std::forward<Args>(args)...);
	}
	//template<>
	void log_parameters(std::shared_ptr<stark::core::Logger> logger, const std::string& constraint, const int idx, const std::string& constraint_name) {
		// Base case for variadic template
	}
}