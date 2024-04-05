#include "EnergyPrescribedPositions.h"

#include "../../time_integration.h"
#include "../../../utils/include.h"


stark::EnergyPrescribedPositions::EnergyPrescribedPositions(core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	// Callbacks
	stark.callbacks.add_is_converged_state_valid([&]() { return this->_is_converged_state_valid(stark); });

	// Declare the energy
	stark.global_energy.add_energy("EnergyPrescribedPositions", this->conn,
		[&](symx::Energy& energy, symx::Element& node)
		{
			// Create symbols
			symx::Vector v1 = energy.make_dof_vector(this->dyn->dof, this->dyn->v1.data, node["point"]);
			symx::Vector x0 = energy.make_vector(this->dyn->x0.data, node["point"]);
			symx::Vector x1_prescribed = energy.make_vector(this->target_positions, node["idx"]);
			symx::Scalar k = energy.make_scalar(this->stiffness, node["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			// Time integration
			symx::Vector x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1 - x1_prescribed).squared_norm();
			energy.set(E);
		}
	);
}
stark::EnergyPrescribedPositions::Handler stark::EnergyPrescribedPositions::add(const PointSetHandler& set, const std::vector<int>& points, const Params& params)
{
	set.exit_if_not_valid("EnergyPrescribedPositions::add");
	const int group = (int)this->stiffness.size();
	this->stiffness.push_back(params.stiffness);
	this->tolerance.push_back(params.tolerance);

	const int begin = (int)this->target_positions.size();
	for (int i = 0; i < (int)points.size(); i++) {
		const int glob_idx = set.get_global_index(points[i]);
		this->target_positions.push_back(this->dyn->x1[glob_idx]);
		this->rest_positions.push_back(this->dyn->x1[glob_idx]);
		this->conn.numbered_push_back({ glob_idx, group });
	}
	const int end = (int)this->target_positions.size();
	this->group_begin_end.push_back({ begin, end });

	return Handler(this, group);
}
stark::EnergyPrescribedPositions::Handler stark::EnergyPrescribedPositions::add_inside_aabb(const PointSetHandler& set, const Eigen::Vector3d& aabb_center, const Eigen::Vector3d& aabb_dim, const Params& params)
{
	set.exit_if_not_valid("EnergyPrescribedPositions::add_inside_aabb");
	Eigen::AlignedBox3d aabb(aabb_center - 0.5*aabb_dim, aabb_center + 0.5*aabb_dim);

	std::vector<int> points;
	for (int i = 0; i < set.size(); i++) {
		if (aabb.contains(set.get_position(i))) {
			points.push_back(i);
		}
	}
	return this->add(set, points, params);
}
stark::EnergyPrescribedPositions::Handler stark::EnergyPrescribedPositions::add_outside_aabb(const PointSetHandler& set, const Eigen::Vector3d& aabb_center, const Eigen::Vector3d& aabb_dim, const Params& params)
{
	set.exit_if_not_valid("EnergyPrescribedPositions::add_outside_aabb");
	Eigen::AlignedBox3d aabb(aabb_center - 0.5*aabb_dim, aabb_center + 0.5*aabb_dim);

	std::vector<int> points;
	for (int i = 0; i < set.size(); i++) {
		if (!aabb.contains(set.get_position(i))) {
			points.push_back(i);
		}
	}
	return this->add(set, points, params);
}
stark::EnergyPrescribedPositions::Params stark::EnergyPrescribedPositions::get_params(const Handler& handler) const
{
	handler.exit_if_not_valid("EnergyPrescribedPositions::get_params");

	const int group = handler.get_idx();
	if (group < 0 || group >= (int)this->stiffness.size()) {
		std::cout << "Stark error: EnergyPrescribedPositions::get_params() found an invalid index." << std::endl;
		exit(-1);
	}

	Params params;
	params.stiffness = this->stiffness[group];
	params.tolerance = this->tolerance[group];
	return params;
}
void stark::EnergyPrescribedPositions::set_params(const Handler& handler, const Params& params)
{
	handler.exit_if_not_valid("EnergyPrescribedPositions::set_params");

	const int group = handler.get_idx();
	if (group < 0 || group >= (int)this->stiffness.size()) {
		std::cout << "Stark error: EnergyPrescribedPositions::set_params() found an invalid index." << std::endl;
		exit(-1);
	}

	this->stiffness[group] = params.stiffness;
	this->tolerance[group] = params.tolerance;
}
void stark::EnergyPrescribedPositions::set_transformation(const Handler& handler, const Eigen::Vector3d& t, const Eigen::Matrix3d& R)
{
	handler.exit_if_not_valid("EnergyPrescribedPositions::set_transformation");
	const int group = handler.get_idx();
	const auto [begin, end] = this->group_begin_end[group];
	for (int i = begin; i < end; i++) {
		this->target_positions[i] = R * this->rest_positions[i] + t;
	}
}

void stark::EnergyPrescribedPositions::set_transformation(const Handler& handler, const Eigen::Vector3d& t, const double angle_deg, const Eigen::Vector3d& axis)
{
	const Eigen::Matrix3d R = Eigen::AngleAxisd(deg2rad(angle_deg), axis.normalized()).toRotationMatrix();
	this->set_transformation(handler, t, R);
}

void stark::EnergyPrescribedPositions::set_target_position(const Handler& handler, int prescribed_idx, const Eigen::Vector3d& t)
{
	handler.exit_if_not_valid("EnergyPrescribedPositions::set_target_position");
	const int group = handler.get_idx();
	const auto [begin, end] = this->group_begin_end[group];
	const int idx = begin + prescribed_idx;
	this->target_positions[idx] = t;
}

bool stark::EnergyPrescribedPositions::_is_converged_state_valid(stark::core::Stark& stark)
{
	const double dt = stark.dt;
	bool is_valid = true;

	auto label_idx = this->conn.get_label_indices({ "idx", "point", "group" });
	for (int i = 0; i < (int)this->conn.size(); i++) {
		auto [idx, point_idx, group] = this->conn.get(i, label_idx);
		const Eigen::Vector3d& x1 = this->dyn->get_x1(point_idx, dt);
		const Eigen::Vector3d& x1_prescribed = this->target_positions[idx];
		const double sq_distance = (x1 - x1_prescribed).squaredNorm();
		const double tol = this->tolerance[group];
		if (sq_distance > tol*tol) {
			is_valid = false;
			this->stiffness[group] *= 2.0;
			break;
		}
	}

	if (!is_valid) {
		stark.console.add_error_msg("Deformable prescribed position constraints are not within tolerance. Stiffness hardened.");
	}

	return is_valid;
}
