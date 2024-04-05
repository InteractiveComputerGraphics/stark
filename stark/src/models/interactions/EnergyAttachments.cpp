#include "EnergyAttachments.h"

#include "../../utils/include.h"
#include "../time_integration.h"
#include "../rigidbodies/rigidbody_transformations.h"
#include <tmd/TriangleMeshDistance.h>


stark::EnergyAttachments::EnergyAttachments(core::Stark& stark, const spPointDynamics dyn, const spRigidBodyDynamics rb)
	: dyn(dyn), rb(rb)
{
	// Callbacks
	stark.callbacks.add_is_converged_state_valid([&]() { return this->_is_converged_state_valid(stark); });

	// Declare the energies
	stark.global_energy.add_energy("EnergyAttachments_d_d_p_p", this->conn_d_d_p_p,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto nodes = { conn["a"], conn["b"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, nodes);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, nodes);
			symx::Scalar k = energy.make_scalar(this->stiffness_d_d_p_p, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1[1] - x1[0]).squared_norm();
			energy.set(E);
		}
	);

	stark.global_energy.add_energy("EnergyAttachments_d_d_p_e", this->conn_d_d_p_e,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto nodes = { conn["p"], conn["e0"], conn["e1"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, nodes);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, nodes);
			symx::Vector bary = energy.make_vector(this->bary_p_e, conn["idx"]);
			symx::Scalar k = energy.make_scalar(this->stiffness_d_d_p_e, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Projection
			symx::Vector p = x1[0];
			symx::Vector q = bary[0] * x1[1] + bary[1] * x1[2];

			// Energy
			symx::Scalar E = 0.5 * k * (q - p).squared_norm();
			energy.set(E);
		}
	);

	stark.global_energy.add_energy("EnergyAttachments_d_d_p_t", this->conn_d_d_p_t,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto nodes = { conn["p"], conn["t0"], conn["t1"], conn["t2"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, nodes);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, nodes);
			symx::Vector bary = energy.make_vector(this->bary_p_t, conn["idx"]);
			symx::Scalar k = energy.make_scalar(this->stiffness_d_d_p_t, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Projection
			symx::Vector p = x1[0];
			symx::Vector q = bary[0] * x1[1] + bary[1] * x1[2] + bary[2] * x1[3];

			// Energy
			symx::Scalar E = 0.5 * k * (q - p).squared_norm();
			energy.set(E);
		}
	);

	stark.global_energy.add_energy("EnergyAttachments_d_d_e_e", this->conn_d_d_e_e,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto nodes = { conn["ea0"], conn["ea1"], conn["eb0"], conn["eb1"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, nodes);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, nodes);
			symx::Vector bary_0 = energy.make_vector(this->bary_e_e_0, conn["idx"]);
			symx::Vector bary_1 = energy.make_vector(this->bary_e_e_1, conn["idx"]);
			symx::Scalar k = energy.make_scalar(this->stiffness_d_d_e_e, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Projection
			symx::Vector p = bary_0[0] * x1[0] + bary_0[1] * x1[1];
			symx::Vector q = bary_1[0] * x1[2] + bary_1[1] * x1[3];

			// Energy
			symx::Scalar E = 0.5 * k * (q - p).squared_norm();
			energy.set(E);
		}
	);

	stark.global_energy.add_energy("EnergyAttachments_rb_d", this->conn_rb_d,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Create symbols
			symx::Scalar k = energy.make_scalar(this->stiffness_rb_d, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			//// Deformable
			symx::Vector v1_d = energy.make_dof_vector(this->dyn->dof, this->dyn->v1.data, conn["p"]);
			symx::Vector x0_d = energy.make_vector(this->dyn->x0.data, conn["p"]);

			//// Rigid body
			symx::Vector x_loc = energy.make_vector(this->rb_points_loc, conn["idx"]);
			symx::Vector x1_rb = this->rb->get_x1(energy, conn["rb"], x_loc, dt);

			// Time integration
			symx::Vector x1_d = time_integration(x0_d, v1_d, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1_d - x1_rb).squared_norm();
			energy.set(E);
		}
	);
}

stark::EnergyAttachments::Handler stark::EnergyAttachments::add(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<int>& points_0, const std::vector<int>& points_1, const Params& params)
{
	set_0.exit_if_not_valid("EnergyAttachments::add");
	set_1.exit_if_not_valid("EnergyAttachments::add");

	if (points_0.size() != points_1.size()) {
		std::cout << "Stark error: EnergyAttachments::add() found an invalid number of points." << std::endl;
		exit(-1);
	}

	const int group = (int)this->stiffness_d_d_p_p.size();
	this->stiffness_d_d_p_p.push_back(params.stiffness);
	this->tolerance_d_d_p_p.push_back(params.tolerance);
	for (int i = 0; i < (int)points_0.size(); i++) {
		this->conn_d_d_p_p.push_back({ group, set_0.get_global_index(points_0[i]), set_1.get_global_index(points_1[i]) });
	}

	const int handler_idx = (int)this->handlers_map.size();
	this->handlers_map.push_back({ AttachmentType::Deformable_Deformable_Point_Point, group });
	return Handler(this, handler_idx);
}
stark::EnergyAttachments::Handler stark::EnergyAttachments::add(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<int>& points, const std::vector<std::array<int, 2>>& edges, const std::vector<std::array<double, 2>>& bary, const Params& params)
{
	set_0.exit_if_not_valid("EnergyAttachments::add");
	set_1.exit_if_not_valid("EnergyAttachments::add");

	const int n = (int)points.size();
	if (edges.size() != n || bary.size() != n) {
		std::cout << "Stark error: EnergyAttachments::add() found an invalid input sizes." << std::endl;
		exit(-1);
	}

	const int group = (int)this->stiffness_d_d_p_e.size();
	this->stiffness_d_d_p_e.push_back(params.stiffness);
	this->tolerance_d_d_p_e.push_back(params.tolerance);
	for (int i = 0; i < (int)edges.size(); i++) {
		this->conn_d_d_p_e.numbered_push_back({ group, set_0.get_global_index(points[i]), set_1.get_global_index(edges[i][0]), set_1.get_global_index(edges[i][1]) });
		this->bary_p_e.push_back({ bary[i][0], bary[i][1] });
	}

	const int handler_idx = (int)this->handlers_map.size();
	this->handlers_map.push_back({ AttachmentType::Deformable_Deformable_Point_Edge, group });
	return Handler(this, handler_idx);
}
stark::EnergyAttachments::Handler stark::EnergyAttachments::add(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<int>& points, const std::vector<std::array<int, 3>>& triangles, const std::vector<std::array<double, 3>>& bary, const Params& params)
{
	set_0.exit_if_not_valid("EnergyAttachments::add");
	set_1.exit_if_not_valid("EnergyAttachments::add");

	const int n = (int)points.size();
	if (triangles.size() != n || bary.size() != n) {
		std::cout << "Stark error: EnergyAttachments::add() found an invalid input sizes." << std::endl;
		exit(-1);
	}

	const int group = (int)this->stiffness_d_d_p_t.size();
	this->stiffness_d_d_p_t.push_back(params.stiffness);
	this->tolerance_d_d_p_t.push_back(params.tolerance);
	for (int i = 0; i < (int)triangles.size(); i++) {
		this->conn_d_d_p_t.numbered_push_back({ group, set_0.get_global_index(points[i]), set_1.get_global_index(triangles[i][0]), set_1.get_global_index(triangles[i][1]), set_1.get_global_index(triangles[i][2]) });
		this->bary_p_t.push_back({ bary[i][0], bary[i][1], bary[i][2] });
	}

	const int handler_idx = (int)this->handlers_map.size();
	this->handlers_map.push_back({ AttachmentType::Deformable_Deformable_Point_Triangle, group });
	return Handler(this, handler_idx);
}
stark::EnergyAttachments::Handler stark::EnergyAttachments::add(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<std::array<int, 2>>& edges_0, const std::vector<std::array<int, 2>>& edges_1, const std::vector<std::array<double, 2>>& bary_0, const std::vector<std::array<double, 2>>& bary_1, const Params& params)
{
	set_0.exit_if_not_valid("EnergyAttachments::add");
	set_1.exit_if_not_valid("EnergyAttachments::add");

	const int n = (int)edges_0.size();
	if (edges_1.size() != n || bary_0.size() != n || bary_1.size() != n) {
		std::cout << "Stark error: EnergyAttachments::add() found an invalid input sizes." << std::endl;
		exit(-1);
	}

	const int group = (int)this->stiffness_d_d_e_e.size();
	this->stiffness_d_d_e_e.push_back(params.stiffness);
	this->tolerance_d_d_e_e.push_back(params.tolerance);
	for (int i = 0; i < n; i++) {
		this->conn_d_d_e_e.numbered_push_back({ group, set_0.get_global_index(edges_0[i][0]), set_0.get_global_index(edges_0[i][1]), set_1.get_global_index(edges_1[i][0]), set_1.get_global_index(edges_1[i][1]) });
		this->bary_e_e_0.push_back({ bary_0[i][0], bary_0[i][1] });
		this->bary_e_e_1.push_back({ bary_1[i][0], bary_1[i][1] });
	}

	const int handler_idx = (int)this->handlers_map.size();
	this->handlers_map.push_back({ AttachmentType::Deformable_Deformable_Edge_Edge, group });
	return Handler(this, handler_idx);
}
stark::EnergyAttachments::MultiHandler stark::EnergyAttachments::add_by_distance(const PointSetHandler& set_0, const PointSetHandler& set_1, const std::vector<int>& points, const std::vector<std::array<int, 3>>& triangles, const double distance, const Params& params)
{
	set_0.exit_if_not_valid("EnergyAttachments::add");
	set_1.exit_if_not_valid("EnergyAttachments::add");

	// Build the triangle mesh distance
	std::vector<Eigen::Vector3d> tmd_vertices = this->dyn->x1.get_set(set_1.get_idx());
	tmd::TriangleMeshDistance mesh_distance(tmd_vertices, triangles);

	// Buffers
	std::vector<int> p_p_points_0;
	std::vector<int> p_p_points_1;
	std::vector<int> p_e_points;
	std::vector<std::array<int, 2>> p_e_edges;
	std::vector<std::array<double, 2>> p_e_bary;
	std::vector<int> p_t_points;
	std::vector<std::array<int, 3>> p_t_triangles;
	std::vector<std::array<double, 3>> p_t_bary;

	// Find points closer than the distance
	for (const int loc : points) {
		const Eigen::Vector3d p = set_0.get_position(loc);
		const auto d = mesh_distance.unsigned_distance(p);
		if (d.distance < distance) {

			const std::array<int, 3> triangle = triangles[d.triangle_id];
			switch (d.nearest_entity)
			{
			case tmd::NearestEntity::V0:
				p_p_points_0.push_back(loc);
				p_p_points_1.push_back(triangle[0]);
				break;
			case tmd::NearestEntity::V1:
				p_p_points_0.push_back(loc);
				p_p_points_1.push_back(triangle[1]);
				break;
			case tmd::NearestEntity::V2:
				p_p_points_0.push_back(loc);
				p_p_points_1.push_back(triangle[2]);
				break;
			case tmd::NearestEntity::E01:
				p_e_points.push_back(loc);
				p_e_edges.push_back({ triangle[0], triangle[1] });
				p_e_bary.push_back({ d.barycentric[0], d.barycentric[1] });
				break;
			case tmd::NearestEntity::E12:
				p_e_points.push_back(loc);
				p_e_edges.push_back({ triangle[1], triangle[2] });
				p_e_bary.push_back({ d.barycentric[1], d.barycentric[2] });
				break;
			case tmd::NearestEntity::E02:
				p_e_points.push_back(loc);
				p_e_edges.push_back({ triangle[0], triangle[2] });
				p_e_bary.push_back({ d.barycentric[0], d.barycentric[2] });
				break;
			case tmd::NearestEntity::F:
				p_t_points.push_back(loc);
				p_t_triangles.push_back(triangle);
				p_t_bary.push_back({ d.barycentric[0], d.barycentric[1], d.barycentric[2] });
				break;

			default:
				break;
			}
		}
	}

	// Add attachments
	return MultiHandler({
		this->add(set_0, set_1, p_p_points_0, p_p_points_1, params),
		this->add(set_0, set_1, p_e_points, p_e_edges, p_e_bary, params),
		this->add(set_0, set_1, p_t_points, p_t_triangles, p_t_bary, params)
	});
}
stark::EnergyAttachments::Handler stark::EnergyAttachments::add(const RigidBodyHandler& rb, const PointSetHandler& set, const std::vector<Eigen::Vector3d>& rb_points_loc, const std::vector<int>& set_points, const Params& params)
{
	rb.exit_if_not_valid("EnergyAttachments::add");
	set.exit_if_not_valid("EnergyAttachments::add");

	const int group_rb_d = (int)this->stiffness_rb_d.size();
	this->stiffness_rb_d.push_back(params.stiffness);
	this->tolerance_rb_d.push_back(params.tolerance);
	const int rb_idx = rb.get_idx();
	for (int i = 0; i < (int)set_points.size(); i++) {
		const int global = set.get_global_index(set_points[i]);
		this->conn_rb_d.numbered_push_back({ group_rb_d, rb_idx, global });
		this->rb_points_loc.push_back(rb_points_loc[i]);
	}

	const int handler_idx = (int)this->handlers_map.size();
	this->handlers_map.push_back({ AttachmentType::Rigid_Deformable, group_rb_d });
	return Handler(this, handler_idx);
}
stark::EnergyAttachments::Handler stark::EnergyAttachments::add(const RigidBodyHandler& rb, const PointSetHandler& set, const std::vector<int>& points, const Params& params)
{
	rb.exit_if_not_valid("EnergyAttachments::add");
	set.exit_if_not_valid("EnergyAttachments::add");

	std::vector<Eigen::Vector3d> rb_points_loc(points.size());
	for (int i = 0; i < (int)points.size(); i++) {
		const int global = set.get_global_index(points[i]);
		rb_points_loc[i] = rb.transform_global_to_local_point(this->dyn->x1[global]);
	}
	return this->add(rb, set, rb_points_loc, points, params);
}
stark::EnergyAttachments::Handler stark::EnergyAttachments::add_by_distance(const RigidBodyHandler& rb, const PointSetHandler& set, const std::vector<Eigen::Vector3d>& loc_vertices, const std::vector<std::array<int, 3>>& triangles, const std::vector<int>& set_points, const double distance, const Params& params)
{
	rb.exit_if_not_valid("EnergyAttachments::add");
	set.exit_if_not_valid("EnergyAttachments::add");

	// Build the triangle mesh distance
	const std::vector<Eigen::Vector3d> glob_vertices = rb.transform_local_to_global_points(loc_vertices);
	tmd::TriangleMeshDistance mesh_distance(glob_vertices, triangles);

	// Buffers
	std::vector<int> rb_d_set_points;
	std::vector<Eigen::Vector3d> rb_points_loc;

	// Find points closer than the distance
	for (const int loc : set_points) {
		const Eigen::Vector3d p = set.get_position(loc);
		const auto d = mesh_distance.unsigned_distance(p);
		if (d.distance < distance) {
			auto q = d.nearest_point;
			rb_d_set_points.push_back(loc);
			rb_points_loc.push_back(rb.transform_global_to_local_point({q[0], q[1], q[2]}));
		}
	}

	// Add attachments
	return this->add(rb, set, rb_points_loc, rb_d_set_points, params);
}
stark::EnergyAttachments::Params stark::EnergyAttachments::get_params(const Handler& handler) const
{
	handler.exit_if_not_valid("EnergyAttachments::get_params");

	const int glob_group = handler.get_idx();
	const AttachmentType att_type = this->handlers_map[glob_group].first;
	const int group = this->handlers_map[glob_group].second;
	switch (att_type) {
	case AttachmentType::Deformable_Deformable_Point_Point:
		return Params().set_stiffness(this->stiffness_d_d_p_p[group]).set_tolerance(this->tolerance_d_d_p_p[group]);
	case AttachmentType::Deformable_Deformable_Point_Edge:
		return Params().set_stiffness(this->stiffness_d_d_p_e[group]).set_tolerance(this->tolerance_d_d_p_e[group]);
	case AttachmentType::Deformable_Deformable_Point_Triangle:
		return Params().set_stiffness(this->stiffness_d_d_p_t[group]).set_tolerance(this->tolerance_d_d_p_t[group]);
	case AttachmentType::Deformable_Deformable_Edge_Edge:
		return Params().set_stiffness(this->stiffness_d_d_e_e[group]).set_tolerance(this->tolerance_d_d_e_e[group]);
	case AttachmentType::Rigid_Deformable:
		return Params().set_stiffness(this->stiffness_rb_d[group]).set_tolerance(this->tolerance_rb_d[group]);
	default:
		std::cout << "Stark error: EnergyAttachments::get_params() found an invalid handler type." << std::endl;
		exit(-1);
	}
}
void stark::EnergyAttachments::set_params(const Handler& handler, const Params& params)
{
	handler.exit_if_not_valid("EnergyAttachments::set_params");

	const int glob_group = handler.get_idx();
	const AttachmentType att_type = this->handlers_map[glob_group].first;
	const int group = this->handlers_map[glob_group].second;
	switch (att_type) {
	case AttachmentType::Deformable_Deformable_Point_Point:
		this->stiffness_d_d_p_p[group] = params.stiffness;
		this->tolerance_d_d_p_p[group] = params.tolerance;
		break;
	case AttachmentType::Deformable_Deformable_Point_Edge:
		this->stiffness_d_d_p_e[group] = params.stiffness;
		this->tolerance_d_d_p_e[group] = params.tolerance;
		break;
	case AttachmentType::Deformable_Deformable_Point_Triangle:
		this->stiffness_d_d_p_t[group] = params.stiffness;
		this->tolerance_d_d_p_t[group] = params.tolerance;
		break;
	case AttachmentType::Deformable_Deformable_Edge_Edge:
		this->stiffness_d_d_e_e[group] = params.stiffness;
		this->tolerance_d_d_e_e[group] = params.tolerance;
		break;
	case AttachmentType::Rigid_Deformable:
		this->stiffness_rb_d[group] = params.stiffness;
		this->tolerance_rb_d[group] = params.tolerance;
		break;
	default:
		std::cout << "Stark error: EnergyAttachments::set_params() found an invalid handler type." << std::endl;
		exit(-1);
	}
}

bool stark::EnergyAttachments::_is_converged_state_valid(core::Stark& stark)
{
	const double dt = stark.dt;
	bool is_valid = true;

	// Deformable-deformable
	{
		// Point - Point
		{
			auto label_idx = this->conn_d_d_p_p.get_label_indices({ "group", "a", "b" });
			for (int i = 0; i < (int)this->conn_d_d_p_p.size(); i++) {
				auto [group, a, b] = this->conn_d_d_p_p.get(i, label_idx);
				const Eigen::Vector3d& x1_a = this->dyn->get_x1(a, dt);
				const Eigen::Vector3d& x1_b = this->dyn->get_x1(b, dt);
				const double sq_distance = (x1_a - x1_b).squaredNorm();
				const double tol = this->tolerance_d_d_p_p[group];
				if (sq_distance > tol*tol) {
					is_valid = false;
					this->stiffness_d_d_p_p[group] *= 2.0;
					break;
				}
			}
		}

		// Point - Edge
		{
			auto label_idx = this->conn_d_d_p_e.get_label_indices({ "idx", "group", "p", "e0", "e1" });
			for (int i = 0; i < (int)this->conn_d_d_p_e.size(); i++) {
				auto [idx, group, p, e0, e1] = this->conn_d_d_p_e.get(i, label_idx);
				const Eigen::Vector3d& x1_p = this->dyn->get_x1(p, dt);
				const Eigen::Vector3d& x1_e0 = this->dyn->get_x1(e0, dt);
				const Eigen::Vector3d& x1_e1 = this->dyn->get_x1(e1, dt);
				const std::array<double, 2>& bary = this->bary_p_e[idx];
				const Eigen::Vector3d x1_e = bary[0] * x1_e0 + bary[1] * x1_e1;
				const double sq_distance = (x1_p - x1_e).squaredNorm();
				const double tol = this->tolerance_d_d_p_e[group];
				if (sq_distance > tol * tol) {
					is_valid = false;
					this->stiffness_d_d_p_e[group] *= 2.0;
					break;
				}
			}
		}

		// Point - Triangle
		{
			auto label_idx = this->conn_d_d_p_t.get_label_indices({ "idx", "group", "p", "t0", "t1", "t2" });
			for (int i = 0; i < (int)this->conn_d_d_p_t.size(); i++) {
				auto [idx, group, p, t0, t1, t2] = this->conn_d_d_p_t.get(i, label_idx);
				const Eigen::Vector3d& x1_p = this->dyn->get_x1(p, dt);
				const Eigen::Vector3d& x1_t0 = this->dyn->get_x1(t0, dt);
				const Eigen::Vector3d& x1_t1 = this->dyn->get_x1(t1, dt);
				const Eigen::Vector3d& x1_t2 = this->dyn->get_x1(t2, dt);
				const std::array<double, 3>& bary = this->bary_p_t[idx];
				const Eigen::Vector3d x1_t = bary[0] * x1_t0 + bary[1] * x1_t1 + bary[2] * x1_t2;
				const double sq_distance = (x1_p - x1_t).squaredNorm();
				const double tol = this->tolerance_d_d_p_t[group];
				if (sq_distance > tol * tol) {
					is_valid = false;
					this->stiffness_d_d_p_t[group] *= 2.0;
					break;
				}
			}
		}

		// Edge - Edge
		{
			auto label_idx = this->conn_d_d_e_e.get_label_indices({ "idx", "group", "ea0", "ea1", "eb0", "eb1" });
			for (int i = 0; i < (int)this->conn_d_d_e_e.size(); i++) {
				auto [idx, group, ea0, ea1, eb0, eb1] = this->conn_d_d_e_e.get(i, label_idx);
				const Eigen::Vector3d& x1_ea0 = this->dyn->get_x1(ea0, dt);
				const Eigen::Vector3d& x1_ea1 = this->dyn->get_x1(ea1, dt);
				const Eigen::Vector3d& x1_eb0 = this->dyn->get_x1(eb0, dt);
				const Eigen::Vector3d& x1_eb1 = this->dyn->get_x1(eb1, dt);
				const std::array<double, 2>& bary_0 = this->bary_e_e_0[idx];
				const std::array<double, 2>& bary_1 = this->bary_e_e_1[idx];
				const Eigen::Vector3d x1_e0 = bary_0[0] * x1_ea0 + bary_0[1] * x1_ea1;
				const Eigen::Vector3d x1_e1 = bary_1[0] * x1_eb0 + bary_1[1] * x1_eb1;
				const double sq_distance = (x1_e0 - x1_e1).squaredNorm();
				const double tol = this->tolerance_d_d_e_e[group];
				if (sq_distance > tol * tol) {
					is_valid = false;
					this->stiffness_d_d_e_e[group] *= 2.0;
					break;
				}
			}
		}
	}

	// Rigid-deformable
	{
		auto label_idx = this->conn_rb_d.get_label_indices({ "idx", "group", "rb", "p" });
		for (int i = 0; i < (int)this->conn_rb_d.size(); i++) {
			auto [idx, group, rb, p] = this->conn_rb_d.get(i, label_idx);
			const Eigen::Vector3d& x1_a = this->rb->get_x1(rb, this->rb_points_loc[idx], dt);
			const Eigen::Vector3d& x1_b = this->dyn->get_x1(p, dt);
			const double sq_distance = (x1_a - x1_b).squaredNorm();
			const double tol = this->tolerance_rb_d[group];
			if (sq_distance > tol*tol) {
				is_valid = false;
				this->stiffness_rb_d[group] *= 2.0;
				break;
			}
		}
	}

	if (!is_valid) {
		stark.console.add_error_msg("Attachment constraints are not within tolerance. Stiffness hardened.");
	}

	return is_valid;
}
