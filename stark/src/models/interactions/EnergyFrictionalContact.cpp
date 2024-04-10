#include "EnergyFrictionalContact.h"

#include <fmt/format.h>

#include "../time_integration.h"
#include "../distances.h"
#include "../rigidbodies/rigidbody_transformations.h"
#include "../../utils/include.h"
#include "friction_geometry.h"

using namespace stark;

EnergyFrictionalContact::EnergyFrictionalContact(core::Stark& stark, const spPointDynamics dyn, const spRigidBodyDynamics rb)
	: dyn(dyn), rb(rb)
{
	if (!stark.settings.simulation.init_frictional_contact) { return; }
	this->is_initialized = true;

	// Callbacks
	stark.callbacks.add_before_time_step([&]() { this->_before_time_step__update_friction_contacts(stark); });
	stark.callbacks.add_before_energy_evaluation([&]() { this->_before_energy_evaluation__update_contacts(stark); });
	stark.callbacks.add_is_intermidiate_state_valid([&]() { return this->_is_intermidiate_state_valid(stark, /*is_initial_check=*/false); });
	stark.callbacks.add_is_initial_state_valid([&]() { return this->_is_intermidiate_state_valid(stark, /*is_initial_check=*/true); });
	stark.callbacks.add_on_intermidiate_state_invalid([&]() { this->_on_intermidiate_state_invalid(stark); });
	stark.callbacks.add_on_time_step_accepted([&]() { this->_on_time_step_accepted(stark); });

	// Contact declarations
	this->_energies_contact_deformables(stark);
	this->_energies_contact_rb(stark);
	this->_energies_contact_rb_deformables(stark);

	// Friction declarations
	this->_energies_friction_deformables(stark);
	this->_energies_friction_rb(stark);
	this->_energies_friction_rb_deformables(stark);
}
EnergyFrictionalContact::GlobalParams EnergyFrictionalContact::get_global_params() const
{
	return this->global_params;
}
void EnergyFrictionalContact::set_global_params(const GlobalParams& params)
{
	this->global_params = params;
	this->contact_stiffness = params.min_contact_stiffness;
}
double stark::EnergyFrictionalContact::get_contact_stiffness() const
{
	return this->contact_stiffness;
}

EnergyFrictionalContact::Handler EnergyFrictionalContact::add_triangles(const PointSetHandler& point_set, const std::vector<std::array<int, 3>>& triangles, const Params& params)
{
	point_set.exit_if_not_valid("EnergyFrictionalContact::add_triangles");
	return this->add_triangles(point_set, triangles, point_set.all(), params);
}
EnergyFrictionalContact::Handler EnergyFrictionalContact::add_triangles(const PointSetHandler& point_set, const std::vector<std::array<int, 3>>& triangles, const std::vector<int>& point_set_map, const Params& params)
{
	point_set.exit_if_not_valid("EnergyFrictionalContact::add_triangles");
	Handler handler = this->_add_triangles_edges_and_points(PhysicalSystem::Deformable, point_set.get_idx(), params, (int)point_set_map.size(), triangles);
	this->_add_deformable(point_set, handler, point_set_map);
	return handler;
}
EnergyFrictionalContact::Handler EnergyFrictionalContact::add_edges(const PointSetHandler& point_set, const std::vector<std::array<int, 2>>& edges, const Params& params)
{
	point_set.exit_if_not_valid("EnergyFrictionalContact::add_edges");
	return this->add_edges(point_set, edges, point_set.all(), params);
}
EnergyFrictionalContact::Handler EnergyFrictionalContact::add_edges(const PointSetHandler& point_set, const std::vector<std::array<int, 2>>& edges, const std::vector<int>& point_set_map, const Params& params)
{
	point_set.exit_if_not_valid("EnergyFrictionalContact::add_edges");
	Handler handler = this->_add_edges_and_points(PhysicalSystem::Deformable, point_set.get_idx(), params, (int)point_set_map.size(), edges);
	this->_add_deformable(point_set, handler, point_set_map);
	return handler;
}
EnergyFrictionalContact::Handler EnergyFrictionalContact::add_triangles(const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Params& params)
{
	rb.exit_if_not_valid("EnergyFrictionalContact::add_triangles");
	Handler handler = this->_add_triangles_edges_and_points(PhysicalSystem::Rigidbody, rb.get_idx(), params, (int)vertices.size(), triangles);
	this->_add_rigid_body(rb, handler, vertices);
	return handler;
}
EnergyFrictionalContact::Handler EnergyFrictionalContact::add_edges(const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 2>>& edges, const Params& params)
{
	rb.exit_if_not_valid("EnergyFrictionalContact::add_triangles");
	Handler handler = this->_add_edges_and_points(PhysicalSystem::Rigidbody, rb.get_idx(), params, (int)vertices.size(), edges);
	this->_add_rigid_body(rb, handler, vertices);
	return handler;
}

void stark::EnergyFrictionalContact::set_contact_thickness(const Handler& obj, const double contact_thickness)
{
	obj.exit_if_not_valid("EnergyFrictionalContact::set_contact_thickness");
	if (contact_thickness <= 0.0) {
		std::cout << "stark error: Contact thickness must be positive in EnergyFrictionalContact." << std::endl;
		exit(-1);
	}
	this->contact_thicknesses[obj.get_idx()] = contact_thickness;
}
void stark::EnergyFrictionalContact::set_friction(const Handler& obj0, const Handler& obj1, const double coulombs_coefficient)
{
	obj0.exit_if_not_valid("EnergyFrictionalContact::set_friction");
	obj1.exit_if_not_valid("EnergyFrictionalContact::set_friction");
	this->pair_coulombs_mu[this->_get_pair_key(obj0, obj1)] = coulombs_coefficient;
}
void stark::EnergyFrictionalContact::disable_collision(const Handler& obj0, const Handler& obj1)
{
	obj0.exit_if_not_valid("EnergyFrictionalContact::disable_collision");
	obj1.exit_if_not_valid("EnergyFrictionalContact::disable_collision");

	const std::array<int, 2> pair = this->_get_pair_key(obj0, obj1);
	if (this->disabled_collision_pairs.find(pair) == this->disabled_collision_pairs.end()) {
		this->disabled_collision_pairs.insert(pair);
		this->pd.add_blacklist(pair[0], pair[1]);
		this->id.add_blacklist(pair[0], pair[1]);
	}
}
bool stark::EnergyFrictionalContact::is_empty() const
{
	return this->meshes.empty();
}


/* ================================================================================= */
/* ===================================  HELPERS  =================================== */
/* ================================================================================= */
std::array<int, 2> stark::EnergyFrictionalContact::_get_pair_key(const Handler& obj0, const Handler& obj1)
{
	return { std::min(obj0.get_idx(), obj1.get_idx()), std::max(obj0.get_idx(), obj1.get_idx()) };
}
double stark::EnergyFrictionalContact::_init_contact_thickness(double contact_thickness)
{
	if (!this->is_initialized) {
		return contact_thickness;
	}

	if (contact_thickness == 0.0) {
		if (this->global_params.default_contact_thickness > 0.0) {
			contact_thickness = this->global_params.default_contact_thickness;
		}
		else {
			std::cout << "stark error: Undefined contact thickness found. Explicitly declare per-object contact thickness or set a global default." << std::endl;
			exit(-1);
		}
	}

	return contact_thickness;
}
EnergyFrictionalContact::Handler EnergyFrictionalContact::_add_edges_and_points(const PhysicalSystem& ps, int idx, const Params& params, int n_vertices, const std::vector<std::array<int, 2>>& edges)
{
	const int group = (int)this->contact_thicknesses.size();

	// Set the contact distance for this group in the array for SymX
	this->contact_thicknesses.push_back(this->_init_contact_thickness(params.contact_thickness));

	// Add to local meshes data structures
	this->meshes.push_back({});
	auto& mesh = this->meshes.back();
	mesh.ps = ps;
	mesh.idx_in_ps = idx;
	mesh.vertices.resize(n_vertices, Eigen::Vector3d::Zero());
	mesh.loc_edges.insert(mesh.loc_edges.end(), edges.begin(), edges.end());
	mesh.loc_triangles.push_back({-1, -1, -1});  // Dummy triangle. We need valid pointer.

	// Add to collision detection
	this->pd.add_mesh(&mesh.vertices[0][0], n_vertices, &mesh.loc_triangles[0][0], /* n_triangles */ 0, &mesh.loc_edges[0][0], (int)mesh.loc_edges.size());
	this->id.add_mesh(&mesh.vertices[0][0], n_vertices, &mesh.loc_triangles[0][0], /* n_triangles */ 0, &mesh.loc_edges[0][0], (int)mesh.loc_edges.size());

	return Handler(this, group);
}
EnergyFrictionalContact::Handler EnergyFrictionalContact::_add_triangles_edges_and_points(const PhysicalSystem& ps, int idx, const Params& params, int n_vertices, const std::vector<std::array<int, 3>>& triangles)
{
	const int group = (int)this->contact_thicknesses.size();

	// Set the contact distance for this group in the array for SymX
	this->contact_thicknesses.push_back(this->_init_contact_thickness(params.contact_thickness));

	// Find edges
	const std::vector<std::array<int, 2>> edges = find_edges_from_simplices(triangles, n_vertices);

	// Add to local meshes data structures
	this->meshes.push_back({});
	auto& mesh = this->meshes.back();
	mesh.ps = ps;
	mesh.idx_in_ps = idx;
	mesh.vertices.resize(n_vertices, Eigen::Vector3d::Zero());
	mesh.loc_edges.insert(mesh.loc_edges.end(), edges.begin(), edges.end());
	mesh.loc_triangles.insert(mesh.loc_triangles.end(), triangles.begin(), triangles.end());

	// Add to collision detection
	this->pd.add_mesh(&mesh.vertices[0][0], n_vertices, &mesh.loc_triangles[0][0], (int)mesh.loc_triangles.size(), &mesh.loc_edges[0][0], (int)mesh.loc_edges.size());
	this->id.add_mesh(&mesh.vertices[0][0], n_vertices, &mesh.loc_triangles[0][0], (int)mesh.loc_triangles.size(), &mesh.loc_edges[0][0], (int)mesh.loc_edges.size());

	return Handler(this, group);
}
void stark::EnergyFrictionalContact::_add_rigid_body(const RigidBodyHandler& rb, const Handler& handler, const std::vector<Eigen::Vector3d>& vertices)
{
	// Bookkeeping
	const int group = handler.get_idx();
	const int rb_group = (int)this->group_to_rigidbody_bookkeeping.size();
	this->group_to_rigidbody_bookkeeping[group] = RigidBodyBookkeeping(group, rb_group, rb.get_idx());

	// Set vertices in local reference for SymX to read
	this->rigidbody_local_vertices.append(vertices);

	// Disable self-collision
	this->disable_collision(handler, handler);
}
void stark::EnergyFrictionalContact::_add_deformable(const PointSetHandler& point_set, const Handler& handler, const std::vector<int>& point_set_map)
{
	// Bookkeeping + store the point_set_map
	const int group = handler.get_idx();
	const int deformable_collision_group = (int)this->group_to_deformable_bookkeeping.size();
	this->group_to_deformable_bookkeeping[group] = DeformableBookkeeping(group, deformable_collision_group, point_set.get_idx(), point_set_map);
}

void EnergyFrictionalContact::_update_vertices(core::Stark& stark, const double dt)
{
	// Note: this runs within the minimization process, positions are updated from prev positions, current velocities and dt

	#pragma omp parallel for schedule(static) num_threads(stark.settings.execution.n_threads)
	for (int group = 0; group < (int)this->meshes.size(); group++) {
		auto& mesh = this->meshes[group];
		const int n = (int)mesh.vertices.size();

		// Physical System
		if (mesh.ps == PhysicalSystem::Deformable) {
			const DeformableBookkeeping& bookkeeping = this->group_to_deformable_bookkeeping[group];
			const std::vector<int>& point_set_map = bookkeeping.point_set_map;
			for (int col_i = 0; col_i < n; col_i++) {
				mesh.vertices[col_i] = this->dyn->get_x1(this->dyn->get_global_index(mesh.idx_in_ps, point_set_map[col_i]), dt);
			}
		}
		else if (mesh.ps == PhysicalSystem::Rigidbody) {
			const Eigen::Vector3d t1 = time_integration(this->rb->t0[mesh.idx_in_ps], this->rb->v1[mesh.idx_in_ps], dt);
			const Eigen::Quaterniond q1 = quat_time_integration(this->rb->q0[mesh.idx_in_ps], this->rb->w1[mesh.idx_in_ps], dt);
			const Eigen::Matrix3d R1 = q1.toRotationMatrix();
			const RigidBodyBookkeeping& bookkeeping = this->group_to_rigidbody_bookkeeping[group];
			for (int col_i = 0; col_i < n; col_i++) {
				mesh.vertices[col_i] = local_to_global_point(this->rigidbody_local_vertices.get(bookkeeping.rb_collision_group, col_i), R1, t1);
			}
		}
		else {
			stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
			exit(-1);
		}
	}
}
const tmcd::ProximityResults& EnergyFrictionalContact::_run_proximity_detection(core::Stark& stark, const double dt)
{
	this->_update_vertices(stark, dt);

	this->pd.set_n_threads(stark.settings.execution.n_threads);
	this->pd.set_edge_edge_parallel_cutoff(this->edge_edge_cross_norm_sq_cutoff);
	this->pd.activate_point_triangle(this->global_params.triangle_point_enabled);
	this->pd.activate_edge_edge(this->global_params.edge_edge_enabled);

	const double max_contact_thickness = *std::max_element(this->contact_thicknesses.begin(), this->contact_thicknesses.end());
	return this->pd.run(2.0*max_contact_thickness);
}
const tmcd::IntersectionResults& EnergyFrictionalContact::_run_intersection_detection(core::Stark& stark, const double dt)
{
	this->_update_vertices(stark, dt);

	this->id.set_n_threads(stark.settings.execution.n_threads);
	return this->id.run();
}

/* ============================================================================================= */
/* ===================================  COLLISION DETECTION  =================================== */
/* ============================================================================================= */
template<std::size_t N>
std::array<int, N> EnergyFrictionalContact::_local_to_ps_global_indices(int group, const std::array<int, N>& local)
{
	const PhysicalSystem ps = this->meshes[group].ps;

	std::array<int, N> global;
	if (ps == PhysicalSystem::Deformable) {
		const DeformableBookkeeping& bookkeeping = this->group_to_deformable_bookkeeping[group];
		const int deformable_idx = bookkeeping.deformable_idx;
		const std::vector<int>& point_set_map = bookkeeping.point_set_map;
		for (int i = 0; i < N; i++) {
			global[i] = this->dyn->get_global_index(deformable_idx, point_set_map[local[i]]);
		}
	}
	else if (ps == PhysicalSystem::Rigidbody) {
		const RigidBodyBookkeeping& bookkeeping = this->group_to_rigidbody_bookkeeping[group];
		const int rb_collision_group = bookkeeping.rb_collision_group;
		for (int i = 0; i < N; i++) {
			global[i] = this->rigidbody_local_vertices.get_global_index(rb_collision_group, local[i]);
		}
	}
	else {
		std::cout << "stark error: Unknown physical system found in EnergyFrictionalContact._local_to_ps_global_indices()\n" << std::endl;
		exit(-1);
	}

	return global;
}
template std::array<int, 1> EnergyFrictionalContact::_local_to_ps_global_indices<1>(int group, const std::array<int, 1>& local);
template std::array<int, 2> EnergyFrictionalContact::_local_to_ps_global_indices<2>(int group, const std::array<int, 2>& local);
template std::array<int, 3> EnergyFrictionalContact::_local_to_ps_global_indices<3>(int group, const std::array<int, 3>& local);
ProximityHelper<1> EnergyFrictionalContact::_get_proximity_helper_point(const tmcd::Point& point)
{
	ProximityHelper<1> helper;
	helper.group = point.set;
	helper.ps = this->meshes[point.set].ps;
	helper.idx_in_ps = this->meshes[point.set].idx_in_ps;
	helper.local_verts = { point.idx };
	helper.verts = this->_local_to_ps_global_indices<1>(helper.group, helper.local_verts);
	return helper;
}
ProximityHelper<2> EnergyFrictionalContact::_get_proximity_helper_edge(const tmcd::TriangleEdge::Edge& edge)
{
	ProximityHelper<2> helper;
	helper.group = edge.set;
	helper.ps = this->meshes[edge.set].ps;
	helper.idx_in_ps = this->meshes[edge.set].idx_in_ps;
	helper.local_verts = edge.vertices;
	helper.verts = this->_local_to_ps_global_indices(helper.group, helper.local_verts);
	return helper;
}
ProximityHelper<3> EnergyFrictionalContact::_get_proximity_helper_triangle(const tmcd::Triangle& triangle)
{
	ProximityHelper<3> helper;
	helper.group = triangle.set;
	helper.ps = this->meshes[triangle.set].ps;
	helper.idx_in_ps = this->meshes[triangle.set].idx_in_ps;
	helper.local_verts = triangle.vertices;
	helper.verts = this->_local_to_ps_global_indices(helper.group, helper.local_verts);
	return helper;
}
ProximityHelper<1> EnergyFrictionalContact::_get_proximity_helper_edge_point(const tmcd::EdgePoint& edge_point)
{
	ProximityHelper<1> helper = this->_get_proximity_helper_point(edge_point.point);
	helper.edge = this->_local_to_ps_global_indices(helper.group, edge_point.edge.vertices);
	return helper;
}
ProximityHelper<2> EnergyFrictionalContact::_get_proximity_helper_edge(const tmcd::Edge& edge)
{
	ProximityHelper<2> helper;
	helper.group = edge.set;
	helper.ps = this->meshes[edge.set].ps;
	helper.idx_in_ps = this->meshes[edge.set].idx_in_ps;
	helper.local_verts = edge.vertices;
	helper.verts = this->_local_to_ps_global_indices(helper.group, helper.local_verts);
	helper.edge = helper.verts;
	return helper;
}
double EnergyFrictionalContact::_get_friction(const int idx0, const int idx1)
{
	const std::array<int, 2> pair = { std::min(idx0, idx1), std::max(idx0, idx1) };
	auto it = this->pair_coulombs_mu.find(pair);
	if (it == this->pair_coulombs_mu.end()) {
		return 0.0;
	}
	else {
		return it->second;
	}
}


/* ======================================================================================== */
/* ===================================  SYMX CALLBACKS  =================================== */
/* ======================================================================================== */
void EnergyFrictionalContact::_before_energy_evaluation__update_contacts(core::Stark& stark)
{
	if (!this->global_params.collisions_enabled) { return; }
	if (this->is_empty()) { return; }

	// Clear contact connectivity
	this->contacts_deformables.clear();
	this->contacts_rb.clear();
	this->contacts_rb_deformables.clear();

	// Run proximity detection
	const auto& proximity = this->_run_proximity_detection(stark, stark.dt);

	// Point - Triangle
	{
		// Point - Point
		for (const auto& pair : proximity.point_triangle.point_point) {
			if (pair.distance > this->_get_contact_distance(pair.first.set, pair.second.point.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<1> B = this->_get_proximity_helper_point(pair.second.point);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.point_triangle.point_point.push_back({ A.group, B.group, A.verts[0], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.point_triangle.point_point.push_back({ A.group, B.group, A.idx_in_ps, B.idx_in_ps, A.verts[0], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.point_triangle.rb_d_point_point.push_back({ A.group, B.group, A.idx_in_ps, A.verts[0], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.point_triangle.rb_d_point_point.push_back({ B.group, A.group, B.idx_in_ps, B.verts[0], A.verts[0] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Edge
		for (const auto& pair : proximity.point_triangle.point_edge) {
			if (pair.distance > this->_get_contact_distance(pair.first.set, pair.second.edge.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second.edge);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.point_triangle.point_edge.push_back({ A.group, B.group, A.verts[0], B.verts[0], B.verts[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.point_triangle.point_edge.push_back({ A.group, B.group, A.idx_in_ps, B.idx_in_ps, A.verts[0], B.verts[0], B.verts[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.point_triangle.rb_d_point_edge.push_back({ A.group, B.group, A.idx_in_ps, A.verts[0], B.verts[0], B.verts[1] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.point_triangle.rb_d_edge_point.push_back({ B.group, A.group, B.idx_in_ps, B.verts[0], B.verts[1], A.verts[0] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Triangle
		for (const auto& pair : proximity.point_triangle.point_triangle) {
			if (pair.distance > this->_get_contact_distance(pair.first.set, pair.second.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<3> B = this->_get_proximity_helper_triangle(pair.second);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.point_triangle.point_triangle.push_back({ A.group, B.group, A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.point_triangle.point_triangle.push_back({ A.group, B.group, A.idx_in_ps, B.idx_in_ps, A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.point_triangle.rb_d_point_triangle.push_back({ A.group, B.group, A.idx_in_ps, A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.point_triangle.rb_d_triangle_point.push_back({ B.group, A.group, B.idx_in_ps, B.verts[0], B.verts[1], B.verts[2], A.verts[0] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}
	} // Point - Triangle

	// Edge - Edge
	{
		//// Point - Point
		for (const auto& pair : proximity.edge_edge.point_point) {
			if (pair.distance > this->_get_contact_distance(pair.first.edge.set, pair.second.edge.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_edge_point(pair.first);
			const ProximityHelper<1> B = this->_get_proximity_helper_edge_point(pair.second);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.edge_edge.point_point.push_back({ A.group, B.group, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.edge_edge.point_point.push_back({ A.group, B.group, A.idx_in_ps, B.idx_in_ps, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.edge_edge.rb_d_point_point.push_back({ A.group, B.group, A.idx_in_ps, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.edge_edge.rb_d_point_point.push_back({ B.group, A.group, B.idx_in_ps, B.edge[0], B.edge[1], B.verts[0], A.edge[0], A.edge[1], A.verts[0] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Edge
		for (const auto& pair : proximity.edge_edge.point_edge) {
			if (pair.distance > this->_get_contact_distance(pair.first.edge.set, pair.second.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_edge_point(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.edge_edge.point_edge.push_back({ A.group, B.group, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.edge_edge.point_edge.push_back({ A.group, B.group, A.idx_in_ps, B.idx_in_ps, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.edge_edge.rb_d_point_edge.push_back({ A.group, B.group, A.idx_in_ps, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.edge_edge.rb_d_edge_point.push_back({ B.group, A.group, B.idx_in_ps, B.edge[0], B.edge[1], A.edge[0], A.edge[1], A.verts[0] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Edge - Edge
		for (const auto& pair : proximity.edge_edge.edge_edge) {
			if (pair.distance > this->_get_contact_distance(pair.first.set, pair.second.set)) { continue; }
			const ProximityHelper<2> A = this->_get_proximity_helper_edge(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.edge_edge.edge_edge.push_back({ A.group, B.group, A.edge[0], A.edge[1], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.edge_edge.edge_edge.push_back({ A.group, B.group, A.idx_in_ps, B.idx_in_ps, A.edge[0], A.edge[1], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.edge_edge.rb_d_edge_edge.push_back({ A.group, B.group, A.idx_in_ps, A.edge[0], A.edge[1], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.edge_edge.rb_d_edge_edge.push_back({ B.group, A.group, B.idx_in_ps, B.edge[0], B.edge[1], A.edge[0], A.edge[1] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}
	} // Edge - Edge
}
void EnergyFrictionalContact::_before_time_step__update_friction_contacts(core::Stark& stark)
{
	if (!this->global_params.collisions_enabled) { return; }
	if (!this->global_params.friction_enabled) { return; }
	if (this->is_empty()) { return; }

	// Clear contact connectivity
	this->friction_deformables.clear();
	this->friction_rb.clear();
	this->friction_rb_deformables.clear();

	// Run proximity detection
	const auto& proximity = this->_run_proximity_detection(stark, /* dt = */ 0.0);

	// Lambdas
	const double k = this->contact_stiffness;
	auto force = [&](double d, int group_a, int group_b) { return this->_barrier_force(d, this->_get_contact_distance(group_a, group_b), k); };
	auto point_point = [&](FrictionContact& contact, const double mu, const double d, const ProximityHelper<1>& A, const ProximityHelper<1>& B)
		{
			const Eigen::Vector3d& P = this->meshes[A.group].vertices[A.local_verts[0]];
			const Eigen::Vector3d& Q = this->meshes[B.group].vertices[B.local_verts[0]];
			contact.T.push_back(projection_matrix_point_point(P, Q));
			contact.mu.push_back(mu);
			contact.fn.push_back(force(d, A.group, B.group));
		};
	auto point_edge = [&](FrictionPointEdge& friction, const double mu, const double d, const ProximityHelper<1>& A, const ProximityHelper<2>& B)
		{
			const Eigen::Vector3d& P = this->meshes[A.group].vertices[A.local_verts[0]];
			const Eigen::Vector3d& E0 = this->meshes[B.group].vertices[B.local_verts[0]];
			const Eigen::Vector3d& E1 = this->meshes[B.group].vertices[B.local_verts[1]];
			friction.bary.push_back(barycentric_point_edge(P, E0, E1));
			friction.contact.T.push_back(projection_matrix_point_edge(P, E0, E1));
			friction.contact.mu.push_back(mu);
			friction.contact.fn.push_back(force(d, A.group, B.group));
		};
	auto point_triangle = [&](FrictionPointTriangle& friction, const double mu, const double d, const ProximityHelper<1>& A, const ProximityHelper<3>& B)
		{
			const Eigen::Vector3d& P = this->meshes[A.group].vertices[A.local_verts[0]];
			const Eigen::Vector3d& T0 = this->meshes[B.group].vertices[B.local_verts[0]];
			const Eigen::Vector3d& T1 = this->meshes[B.group].vertices[B.local_verts[1]];
			const Eigen::Vector3d& T2 = this->meshes[B.group].vertices[B.local_verts[2]];
			friction.bary.push_back(barycentric_point_triangle(P, T0, T1, T2));
			friction.contact.T.push_back(projection_matrix_triangle(T0, T1, T2));
			friction.contact.mu.push_back(mu);
			friction.contact.fn.push_back(force(d, A.group, B.group));
		};
	auto edge_edge = [&](FrictionEdgeEdge& friction, const double mu, const double d, const ProximityHelper<2>& A, const ProximityHelper<2>& B)
		{
			const Eigen::Vector3d& EA0 = this->meshes[A.group].vertices[A.local_verts[0]];
			const Eigen::Vector3d& EA1 = this->meshes[A.group].vertices[A.local_verts[1]];
			const Eigen::Vector3d& EB0 = this->meshes[B.group].vertices[B.local_verts[0]];
			const Eigen::Vector3d& EB1 = this->meshes[B.group].vertices[B.local_verts[1]];
			friction.bary.push_back(barycentric_edge_edge(EA0, EA1, EB0, EB1));
			friction.contact.T.push_back(projection_matrix_edge_edge(EA0, EA1, EB0, EB1));
			friction.contact.mu.push_back(mu);
			friction.contact.fn.push_back(force(d, A.group, B.group));
		};

	// Point - Triangle
	{
		//// Point - Point
		for (const auto& pair : proximity.point_triangle.point_point) {
			if (pair.distance > this->_get_contact_distance(pair.first.set, pair.second.point.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<1> B = this->_get_proximity_helper_point(pair.second.point);
			const double mu = this->_get_friction(A.group, B.group);
			if (mu == 0.0) { continue; }

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->friction_deformables.point_point.conn.numbered_push_back({ A.verts[0], B.verts[0] });
				point_point(this->friction_deformables.point_point.contact, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb.point_point.conn.numbered_push_back({ A.idx_in_ps, B.idx_in_ps, A.verts[0], B.verts[0] });
				point_point(this->friction_rb.point_point.contact, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->friction_rb_deformables.point_point.conn.numbered_push_back({ A.idx_in_ps, A.verts[0], B.verts[0] });
				point_point(this->friction_rb_deformables.point_point.contact, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb_deformables.point_point.conn.numbered_push_back({ B.idx_in_ps, B.verts[0], A.verts[0] });
				point_point(this->friction_rb_deformables.point_point.contact, mu, pair.distance, A, B);
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Edge
		for (const auto& pair : proximity.point_triangle.point_edge) {
			if (pair.distance > this->_get_contact_distance(pair.first.set, pair.second.edge.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second.edge);
			const double mu = this->_get_friction(A.group, B.group);
			if (mu == 0.0) { continue; }

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->friction_deformables.point_edge.conn.numbered_push_back({ A.verts[0], B.verts[0], B.verts[1] });
				point_edge(this->friction_deformables.point_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb.point_edge.conn.numbered_push_back({ A.idx_in_ps, B.idx_in_ps, A.verts[0], B.verts[0], B.verts[1] });
				point_edge(this->friction_rb.point_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->friction_rb_deformables.point_edge.conn.numbered_push_back({ A.idx_in_ps, A.verts[0], B.verts[0], B.verts[1] });
				point_edge(this->friction_rb_deformables.point_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb_deformables.edge_point.conn.numbered_push_back({ B.idx_in_ps, B.verts[0], B.verts[1], A.verts[0] });
				point_edge(this->friction_rb_deformables.edge_point.data, mu, pair.distance, A, B);
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Triangle
		for (const auto& pair : proximity.point_triangle.point_triangle) {
			if (pair.distance > this->_get_contact_distance(pair.first.set, pair.second.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<3> B = this->_get_proximity_helper_triangle(pair.second);
			const double mu = this->_get_friction(A.group, B.group);
			if (mu == 0.0) { continue; }

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->friction_deformables.point_triangle.conn.numbered_push_back({ A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
				point_triangle(this->friction_deformables.point_triangle.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb.point_triangle.conn.numbered_push_back({ A.idx_in_ps, B.idx_in_ps, A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
				point_triangle(this->friction_rb.point_triangle.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->friction_rb_deformables.point_triangle.conn.numbered_push_back({ A.idx_in_ps, A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
				point_triangle(this->friction_rb_deformables.point_triangle.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb_deformables.triangle_point.conn.numbered_push_back({ B.idx_in_ps, B.verts[0], B.verts[1], B.verts[2], A.verts[0] });
				point_triangle(this->friction_rb_deformables.triangle_point.data, mu, pair.distance, A, B);
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}
	} // Point - Triangle

	{  // Edge - Edge
		//// Point - Point
		for (const auto& pair : proximity.edge_edge.point_point) {
			if (pair.distance > this->_get_contact_distance(pair.first.edge.set, pair.second.edge.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_edge_point(pair.first);
			const ProximityHelper<1> B = this->_get_proximity_helper_edge_point(pair.second);
			const double mu = this->_get_friction(A.group, B.group);
			if (mu == 0.0) { continue; }

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->friction_deformables.point_point.conn.numbered_push_back({ A.verts[0], B.verts[0] });
				point_point(this->friction_deformables.point_point.contact, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb.point_point.conn.numbered_push_back({ A.idx_in_ps, B.idx_in_ps, A.verts[0], B.verts[0] });
				point_point(this->friction_rb.point_point.contact, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->friction_rb_deformables.point_point.conn.numbered_push_back({ A.idx_in_ps, A.verts[0], B.verts[0] });
				point_point(this->friction_rb_deformables.point_point.contact, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb_deformables.point_point.conn.numbered_push_back({ B.idx_in_ps, B.verts[0], A.verts[0] });
				point_point(this->friction_rb_deformables.point_point.contact, mu, pair.distance, A, B);
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Edge
		for (const auto& pair : proximity.edge_edge.point_edge) {
			if (pair.distance > this->_get_contact_distance(pair.first.edge.set, pair.second.set)) { continue; }
			const ProximityHelper<1> A = this->_get_proximity_helper_edge_point(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second);
			const double mu = this->_get_friction(A.group, B.group);
			if (mu == 0.0) { continue; }

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->friction_deformables.point_edge.conn.numbered_push_back({ A.verts[0], B.verts[0], B.verts[1] });
				point_edge(this->friction_deformables.point_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb.point_edge.conn.numbered_push_back({ A.idx_in_ps, B.idx_in_ps, A.verts[0], B.verts[0], B.verts[1] });
				point_edge(this->friction_rb.point_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->friction_rb_deformables.point_edge.conn.numbered_push_back({ A.idx_in_ps, A.verts[0], B.verts[0], B.verts[1] });
				point_edge(this->friction_rb_deformables.point_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb_deformables.edge_point.conn.numbered_push_back({ B.idx_in_ps, B.verts[0], B.verts[1], A.verts[0] });
				point_edge(this->friction_rb_deformables.edge_point.data, mu, pair.distance, A, B);
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Edge - Edge
		for (const auto& pair : proximity.edge_edge.edge_edge) {
			if (pair.distance > this->_get_contact_distance(pair.first.set, pair.second.set)) { continue; }
			const ProximityHelper<2> A = this->_get_proximity_helper_edge(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second);
			const double mu = this->_get_friction(A.group, B.group);
			if (mu == 0.0) { continue; }

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->friction_deformables.edge_edge.conn.numbered_push_back({ A.verts[0], A.verts[1], B.verts[0], B.verts[1] });
				edge_edge(this->friction_deformables.edge_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb.edge_edge.conn.numbered_push_back({ A.idx_in_ps, B.idx_in_ps, A.verts[0], A.verts[1], B.verts[0], B.verts[1] });
				edge_edge(this->friction_rb.edge_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->friction_rb_deformables.edge_edge.conn.numbered_push_back({ A.idx_in_ps, A.verts[0], A.verts[1], B.verts[0], B.verts[1] });
				edge_edge(this->friction_rb_deformables.edge_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb_deformables.edge_edge.conn.numbered_push_back({ B.idx_in_ps, B.verts[0], B.verts[1], A.verts[0], A.verts[1] });
				edge_edge(this->friction_rb_deformables.edge_edge.data, mu, pair.distance, A, B);
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}
	}
}
bool EnergyFrictionalContact::_is_intermidiate_state_valid(core::Stark& stark, bool is_initial_check)
{
	if (!this->global_params.collisions_enabled) { return true; }
	if (!this->global_params.intersection_test_enabled) { return true; }
	if (this->is_empty()) { return true; }

	const auto& intersections = this->_run_intersection_detection(stark, stark.dt);
	const bool collision_found = !intersections.edge_triangle.empty();

	if (collision_found && is_initial_check) {
		// Find unique colliding objects
		unordered_array_set<int, 2> colliding_objects;
		for (const auto& pair : intersections.edge_triangle) {
			const std::array<int, 2> pair_set = { std::min(pair.first.set, pair.second.set), std::max(pair.first.set, pair.second.set) };
			colliding_objects.insert(pair_set);
		}
		
		// Print pairs found in collision
		stark.console.print("Stark error: Initial collision detected between the following contact object pairs:\n", core::ConsoleVerbosity::Frames);
		for (const auto& pair : colliding_objects) {
			stark.console.print(fmt::format("\t- {} and {}\n", pair[0], pair[1]), core::ConsoleVerbosity::Frames);
		}
	}

	return !collision_found;
}
void stark::EnergyFrictionalContact::_on_intermidiate_state_invalid(core::Stark& stark)
{
	const double old_stiffness = this->contact_stiffness;
	this->contact_stiffness *= 2.0;
	const double new_stiffness = this->contact_stiffness;
	stark.console.add_error_msg(fmt::format("Penetration couldn't be avoided. Contact stiffness hardened from {:.1e} to {:.1e}.", old_stiffness, new_stiffness));
}
void stark::EnergyFrictionalContact::_on_time_step_accepted(core::Stark& stark)
{
	this->contact_stiffness = std::max(this->global_params.min_contact_stiffness, 0.99*this->contact_stiffness);
}


/* ========================================================================================== */
/* ===================================  SYMX DEFINITIONS  =================================== */
/* ========================================================================================== */
void EnergyFrictionalContact::_energies_contact_deformables(core::Stark& stark)
{
	// Point - Triangle
	//// Point - Point
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "pt_pp"), this->contacts_deformables.point_triangle.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			std::vector<symx::Vector> q = this->_get_d_x1(energy, stark, { conn["q"] });
			symx::Scalar d = distance_point_point(p[0], q[0]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);

	//// Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "pt_pe"), this->contacts_deformables.point_triangle.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			std::vector<symx::Vector> e = this->_get_d_x1(energy, stark, { conn["e0"], conn["e1"] });
			symx::Scalar d = distance_point_line(p[0], e[0], e[1]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);

	//// Point - Triangle
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "pt_pt"), this->contacts_deformables.point_triangle.point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			std::vector<symx::Vector> t = this->_get_d_x1(energy, stark, { conn["t0"], conn["t1"], conn["t2"] });
			symx::Scalar d = distance_point_plane(p[0], t[0], t[1], t[2]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);


	// Edge - Edge
	//// Point - Point
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "ee_pp"), this->contacts_deformables.edge_edge.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto [ea, ea_rest, p] = this->_get_d_edge_point(energy, stark, { conn["ea0"], conn["ea1"], conn["p"] });
			auto [eb, eb_rest, q] = this->_get_d_edge_point(energy, stark, { conn["eb0"], conn["eb1"], conn["q"] });
			symx::Scalar d = distance_point_point(p[0], q[0]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);

	//// Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "ee_pe"), this->contacts_deformables.edge_edge.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto [ea, ea_rest, p] = this->_get_d_edge_point(energy, stark, { conn["ea0"], conn["ea1"], conn["p"] });
			auto [eb, eb_rest] = this->_get_d_edge(energy, stark, { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_point_line(p[0], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);

	//// Edge - Edge
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "ee_ee"), this->contacts_deformables.edge_edge.edge_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto [ea, ea_rest] = this->_get_d_edge(energy, stark, { conn["ea0"], conn["ea1"] });
			auto [eb, eb_rest] = this->_get_d_edge(energy, stark, { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_line_line(ea[0], ea[1], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);
}
void EnergyFrictionalContact::_energies_contact_rb(core::Stark& stark)
{
	// Point - Triangle
	//// Point - Point
	stark.global_energy.add_energy(this->_get_contact_label("rb_rb", "pt_pp"), this->contacts_rb.point_triangle.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["a"], { conn["p"] });
			std::vector<symx::Vector> q = this->_get_rb_x1(energy, stark, conn["b"], { conn["q"] });
			symx::Scalar d = distance_point_point(p[0], q[0]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);

	//// Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_rb", "pt_pe"), this->contacts_rb.point_triangle.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["a"], { conn["p"] });
			std::vector<symx::Vector> e = this->_get_rb_x1(energy, stark, conn["b"], { conn["e0"], conn["e1"] });
			symx::Scalar d = distance_point_line(p[0], e[0], e[1]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);

	//// Point - Triangle
	stark.global_energy.add_energy(this->_get_contact_label("rb_rb", "pt_pt"), this->contacts_rb.point_triangle.point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["a"], { conn["p"] });
			std::vector<symx::Vector> t = this->_get_rb_x1(energy, stark, conn["b"], { conn["t0"], conn["t1"], conn["t2"] });
			symx::Scalar d = distance_point_plane(p[0], t[0], t[1], t[2]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);

	// Edge - Edge
	//// Point - Point
	stark.global_energy.add_energy(this->_get_contact_label("rb_rb", "ee_pp"), this->contacts_rb.edge_edge.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			energy.disable_check_for_duplicate_dofs();
			auto [ea, ea_rest, p] = this->_get_rb_edge_point(energy, stark, conn["a"], { conn["ea0"], conn["ea1"], conn["p"] });
			auto [eb, eb_rest, q] = this->_get_rb_edge_point(energy, stark, conn["b"], { conn["eb0"], conn["eb1"], conn["q"] });
			symx::Scalar d = distance_point_point(p[0], q[0]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);

	//// Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_rb", "ee_pe"), this->contacts_rb.edge_edge.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			energy.disable_check_for_duplicate_dofs();
			auto [ea, ea_rest, p] = this->_get_rb_edge_point(energy, stark, conn["a"], { conn["ea0"], conn["ea1"], conn["p"] });
			auto [eb, eb_rest] = this->_get_rb_edge(energy, stark, conn["b"], { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_point_line(p[0], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);

	//// Edge - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_rb", "ee_ee"), this->contacts_rb.edge_edge.edge_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			energy.disable_check_for_duplicate_dofs();
			auto [ea, ea_rest] = this->_get_rb_edge(energy, stark, conn["a"], { conn["ea0"], conn["ea1"] });
			auto [eb, eb_rest] = this->_get_rb_edge(energy, stark, conn["b"], { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_line_line(ea[0], ea[1], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);
}
void EnergyFrictionalContact::_energies_contact_rb_deformables(core::Stark& stark)
{
	// Point - Triangle
	//// RB -> D: Point - Point
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_pp"), this->contacts_rb_deformables.point_triangle.rb_d_point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["rb"], { conn["p"] });
			std::vector<symx::Vector> q = this->_get_d_x1(energy, stark, { conn["q"] });
			symx::Scalar d = distance_point_point(p[0], q[0]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);

	//// RB -> D: Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_pe"), this->contacts_rb_deformables.point_triangle.rb_d_point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["rb"], { conn["p"] });
			std::vector<symx::Vector> e = this->_get_d_x1(energy, stark, { conn["e0"], conn["e1"] });
			symx::Scalar d = distance_point_line(p[0], e[0], e[1]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);

	//// RB -> D: Point - Triangle
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_pt"), this->contacts_rb_deformables.point_triangle.rb_d_point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["rb"], { conn["p"] });
			std::vector<symx::Vector> t = this->_get_d_x1(energy, stark, { conn["t0"], conn["t1"], conn["t2"] });
			symx::Scalar d = distance_point_plane(p[0], t[0], t[1], t[2]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);

	//// D -> RB: Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_ep"), this->contacts_rb_deformables.point_triangle.rb_d_edge_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> e = this->_get_rb_x1(energy, stark, conn["rb"], { conn["e0"], conn["e1"] });
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			symx::Scalar d = distance_point_line(p[0], e[0], e[1]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);

	//// D -> RB: Point - Triangle
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_tp"), this->contacts_rb_deformables.point_triangle.rb_d_triangle_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> t = this->_get_rb_x1(energy, stark, conn["rb"], { conn["t0"], conn["t1"], conn["t2"] });
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			symx::Scalar d = distance_point_plane(p[0], t[0], t[1], t[2]);
			this->_set_barrier_potential(energy, stark, d, conn["group_a"], conn["group_b"]);
		}
	);


	// Edge - Edge
	//// Point - Point
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "ee_pp"), this->contacts_rb_deformables.edge_edge.rb_d_point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			energy.disable_check_for_duplicate_dofs();
			auto [ea, ea_rest, p] = this->_get_rb_edge_point(energy, stark, conn["rb"], { conn["ea0"], conn["ea1"], conn["p"] });
			auto [eb, eb_rest, q] = this->_get_d_edge_point(energy, stark, { conn["eb0"], conn["eb1"], conn["q"] });
			symx::Scalar d = distance_point_point(p[0], q[0]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);

	//// Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "ee_pe"), this->contacts_rb_deformables.edge_edge.rb_d_point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			energy.disable_check_for_duplicate_dofs();
			auto [ea, ea_rest, p] = this->_get_rb_edge_point(energy, stark, conn["rb"], { conn["ea0"], conn["ea1"], conn["p"] });
			auto [eb, eb_rest] = this->_get_d_edge(energy, stark, { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_point_line(p[0], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);

	//// Edge - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "ee_ee"), this->contacts_rb_deformables.edge_edge.rb_d_edge_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto [ea, ea_rest] = this->_get_rb_edge(energy, stark, conn["rb"], { conn["ea0"], conn["ea1"] });
			auto [eb, eb_rest] = this->_get_d_edge(energy, stark, { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_line_line(ea[0], ea[1], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);

	//// D -> RB: Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "ee_ep"), this->contacts_rb_deformables.edge_edge.rb_d_edge_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto [ea, ea_rest] = this->_get_rb_edge(energy, stark, conn["rb"], { conn["ea0"], conn["ea1"] });
			auto [eb, eb_rest, q] = this->_get_d_edge_point(energy, stark, { conn["eb0"], conn["eb1"], conn["q"] });
			symx::Scalar d = distance_point_line(q[0], ea[0], ea[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest, conn["group_a"], conn["group_b"]);
		}
	);
}

void EnergyFrictionalContact::_energies_friction_deformables(core::Stark& stark)
{
	// Point - Point
	stark.global_energy.add_energy(this->_get_friction_label("d_d", "pp"), this->friction_deformables.point_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_d_v1(energy, { conn["p"] });
			std::vector<symx::Vector> vq = this->_get_d_v1(energy, { conn["q"] });
			symx::Vector v = vq[0] - vp[0];
			this->_set_friction_potential(energy, stark, v, conn["idx"], this->friction_deformables.point_point.contact);
		}
	);

	// Point - Edge
	stark.global_energy.add_energy(this->_get_friction_label("d_d", "pe"), this->friction_deformables.point_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_d_v1(energy, { conn["p"] });
			std::vector<symx::Vector> ve = this->_get_d_v1(energy, { conn["e0"], conn["e1"] });
			this->_set_friction_point_edge(energy, stark, vp[0], ve, conn["idx"], this->friction_deformables.point_edge.data);
		}
	);

	// Point - Triangle
	stark.global_energy.add_energy(this->_get_friction_label("d_d", "pt"), this->friction_deformables.point_triangle.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_d_v1(energy, { conn["p"] });
			std::vector<symx::Vector> vt = this->_get_d_v1(energy, { conn["t0"], conn["t1"], conn["t2"] });
			this->_set_friction_point_triangle(energy, stark, vp[0], vt, conn["idx"], this->friction_deformables.point_triangle.data);
		}
	);

	// Edge - Edge
	stark.global_energy.add_energy(this->_get_friction_label("d_d", "ee"), this->friction_deformables.edge_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vea = this->_get_d_v1(energy, { conn["ea0"], conn["ea1"] });
			std::vector<symx::Vector> veb = this->_get_d_v1(energy, { conn["eb0"], conn["eb1"] });
			this->_set_friction_edge_edge(energy, stark, vea, veb, conn["idx"], this->friction_deformables.edge_edge.data);
		}
	);
}
void EnergyFrictionalContact::_energies_friction_rb(core::Stark& stark)
{
	// Point - Point
	stark.global_energy.add_energy(this->_get_friction_label("rb_rb", "pp"), this->friction_rb.point_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_rb_v1(energy, stark, conn["a"], { conn["p"] });
			std::vector<symx::Vector> vq = this->_get_rb_v1(energy, stark, conn["b"], { conn["q"] });
			symx::Vector v = vq[0] - vp[0];
			this->_set_friction_potential(energy, stark, v, conn["idx"], this->friction_rb.point_point.contact);
		}
	);

	// Point - Edge
	stark.global_energy.add_energy(this->_get_friction_label("rb_rb", "pe"), this->friction_rb.point_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_rb_v1(energy, stark, conn["a"], { conn["p"] });
			std::vector<symx::Vector> ve = this->_get_rb_v1(energy, stark, conn["b"], { conn["e0"], conn["e1"] });
			this->_set_friction_point_edge(energy, stark, vp[0], ve, conn["idx"], this->friction_rb.point_edge.data);
		}
	);

	// Point - Triangle
	stark.global_energy.add_energy(this->_get_friction_label("rb_rb", "pt"), this->friction_rb.point_triangle.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_rb_v1(energy, stark, conn["a"], { conn["p"] });
			std::vector<symx::Vector> vt = this->_get_rb_v1(energy, stark, conn["b"], { conn["t0"], conn["t1"], conn["t2"] });
			this->_set_friction_point_triangle(energy, stark, vp[0], vt, conn["idx"], this->friction_rb.point_triangle.data);
		}
	);

	// Edge - Edge
	stark.global_energy.add_energy(this->_get_friction_label("rb_rb", "ee"), this->friction_rb.edge_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vea = this->_get_rb_v1(energy, stark, conn["a"], { conn["ea0"], conn["ea1"] });
			std::vector<symx::Vector> veb = this->_get_rb_v1(energy, stark, conn["b"], { conn["eb0"], conn["eb1"] });
			this->_set_friction_edge_edge(energy, stark, vea, veb, conn["idx"], this->friction_rb.edge_edge.data);
		}
	);
}
void EnergyFrictionalContact::_energies_friction_rb_deformables(core::Stark& stark)
{
	// Point - Point
	stark.global_energy.add_energy(this->_get_friction_label("rb_d", "pp"), this->friction_rb_deformables.point_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_rb_v1(energy, stark, conn["rb"], { conn["p"] });
			std::vector<symx::Vector> vq = this->_get_d_v1(energy, { conn["q"] });

			symx::Vector v = vq[0] - vp[0];
			this->_set_friction_potential(energy, stark, v, conn["idx"], this->friction_rb_deformables.point_point.contact);
		}
	);
	// Point - Edge
	stark.global_energy.add_energy(this->_get_friction_label("rb_d", "pe"), this->friction_rb_deformables.point_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_rb_v1(energy, stark, conn["rb"], { conn["p"] });
			std::vector<symx::Vector> ve = this->_get_d_v1(energy, { conn["e0"], conn["e1"] });
			this->_set_friction_point_edge(energy, stark, vp[0], ve, conn["idx"], this->friction_rb_deformables.point_edge.data);
		}
	);
	// Point - Triangle
	stark.global_energy.add_energy(this->_get_friction_label("rb_d", "pt"), this->friction_rb_deformables.point_triangle.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_rb_v1(energy, stark, conn["rb"], { conn["p"] });
			std::vector<symx::Vector> vt = this->_get_d_v1(energy, { conn["t0"], conn["t1"], conn["t2"] });
			this->_set_friction_point_triangle(energy, stark, vp[0], vt, conn["idx"], this->friction_rb_deformables.point_triangle.data);
		}
	);
	// Edge - Edge
	stark.global_energy.add_energy(this->_get_friction_label("rb_d", "ee"), this->friction_rb_deformables.edge_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vea = this->_get_rb_v1(energy, stark, conn["rb"], { conn["ea0"], conn["ea1"] });
			std::vector<symx::Vector> veb = this->_get_d_v1(energy, { conn["eb0"], conn["eb1"] });
			this->_set_friction_edge_edge(energy, stark, vea, veb, conn["idx"], this->friction_rb_deformables.edge_edge.data);
		}
	);
	// D -> RB: Point - Edge
	stark.global_energy.add_energy(this->_get_friction_label("rb_d", "ep"), this->friction_rb_deformables.edge_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> ve = this->_get_rb_v1(energy, stark, conn["rb"], { conn["e0"], conn["e1"] });
			std::vector<symx::Vector> vp = this->_get_d_v1(energy, { conn["p"] });
			this->_set_friction_point_edge(energy, stark, vp[0], ve, conn["idx"], this->friction_rb_deformables.edge_point.data);
		}
	);
	// D -> RB: Point - Triangle
	stark.global_energy.add_energy(this->_get_friction_label("rb_d", "tp"), this->friction_rb_deformables.triangle_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vt = this->_get_rb_v1(energy, stark, conn["rb"], { conn["t0"], conn["t1"], conn["t2"] });
			std::vector<symx::Vector> vp = this->_get_d_v1(energy, { conn["p"] });
			this->_set_friction_point_triangle(energy, stark, vp[0], vt, conn["idx"], this->friction_rb_deformables.triangle_point.data);
		}
	);
}


/* ============================================================================= */
/* ===================================  IPC  =================================== */
/* ============================================================================= */
symx::Scalar EnergyFrictionalContact::_barrier_potential(const symx::Scalar& d, const symx::Scalar& dhat, const symx::Scalar& k)
{
	if (this->ipc_barrier_type == IPCBarrierType::Cubic) {
		return k * (dhat - d).powN(3) / 3.0;
	}
	else if (this->ipc_barrier_type == IPCBarrierType::Log) {
		return -k * (dhat - d).powN(2) * log(d / dhat);
	}
	else {
		std::cout << "stark error: Unknown IPC barrier type." << std::endl;
		exit(-1);
	}
}
double EnergyFrictionalContact::_barrier_force(const double d, const double dhat, const double k)
{
	if (this->ipc_barrier_type == IPCBarrierType::Cubic) {
		return k * std::pow(dhat - d, 2);
	}
	else if (this->ipc_barrier_type == IPCBarrierType::Log) {
		return (k * (dhat - d) * (2.0 * d * std::log(d / dhat) + d - dhat)) / d;
	}
	else {
		std::cout << "stark error: Unknown IPC barrier type." << std::endl;
		exit(-1);
	}
}
symx::Scalar EnergyFrictionalContact::_edge_edge_mollifier(const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest)
{
	symx::Scalar eps_x = 1e-3 * (ea_rest[0] - ea_rest[1]).squared_norm() * (eb_rest[0] - eb_rest[1]).squared_norm();
	symx::Scalar x = (ea[1] - ea[0]).cross3(eb[1] - eb[0]).squared_norm();
	symx::Scalar x_div_eps_x = x / eps_x;
	symx::Scalar f = (-x_div_eps_x + 2.0) * x_div_eps_x;
	symx::Scalar mollifier = symx::branch(x > eps_x, 1.0, f);
	return mollifier;
}
symx::Scalar EnergyFrictionalContact::_friction_potential(const symx::Vector& v, const symx::Scalar& fn, const symx::Scalar& mu, const symx::Matrix& T, const symx::Scalar& epsv, const symx::Scalar& dt)
{
	constexpr double PERTURBATION = 1e-9;
	symx::Vector vt = T * v;
	symx::Vector ut = vt * dt;
	ut[0] += 1.13 * PERTURBATION;
	ut[1] -= 1.07 * PERTURBATION;
	symx::Scalar u = ut.norm();

	symx::Scalar epsu = dt * epsv;
	if (this->ipc_friction_type == IPCFrictionType::C0) {
		symx::Scalar k = mu * fn / epsu;
		symx::Scalar eps = mu * fn / (2.0 * k);

		symx::Scalar E_stick = 0.5 * k * u.powN(2);
		symx::Scalar E_slide = mu * fn * (u - eps);
		symx::Scalar E = symx::branch(u < epsu, E_stick, E_slide);
		return E;
	}
	else if (this->ipc_friction_type == IPCFrictionType::C1) {
		symx::Scalar E_stick = mu * fn * (-u * u * u / (3.0 * epsu.powN(2)) + u * u / epsu + epsu / 3.0);
		symx::Scalar E_slide = mu * fn * u;
		symx::Scalar E = symx::branch(u < epsu, E_stick, E_slide);
		return E;
	}
	else {
		std::cout << "stark error: Unknown IPC friction type." << std::endl;
		exit(-1);
	}
}



/* ====================================================================================== */
/* ===================================  SYMX SETTERS  =================================== */
/* ====================================================================================== */
symx::Scalar stark::EnergyFrictionalContact::_get_contact_distance(symx::Energy& energy, const symx::Index& group_a, const symx::Index& group_b)
{
	symx::Scalar dhat_a = energy.make_scalar(this->contact_thicknesses, group_a);
	symx::Scalar dhat_b = energy.make_scalar(this->contact_thicknesses, group_b);
	symx::Scalar dhat = dhat_a + dhat_b;
	return dhat;
}
double stark::EnergyFrictionalContact::_get_contact_distance(int group_a, int group_b)
{
	double dhat_a = this->contact_thicknesses[group_a];
	double dhat_b = this->contact_thicknesses[group_b];
	return dhat_a + dhat_b;
}
void EnergyFrictionalContact::_set_barrier_potential(symx::Energy& energy, const core::Stark& stark, const symx::Scalar& d, const symx::Index& group_a, const symx::Index& group_b)
{
	symx::Scalar dhat = this->_get_contact_distance(energy, group_a, group_b);
	symx::Scalar k = energy.make_scalar(this->contact_stiffness);
	symx::Scalar E = this->_barrier_potential(d, dhat, k);
	energy.set(E);
}
void EnergyFrictionalContact::_set_edge_edge_mollified_barrier_potential(symx::Energy& energy, const core::Stark& stark, const symx::Scalar& d, const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest, const symx::Index& group_a, const symx::Index& group_b)
{
	symx::Scalar k = energy.make_scalar(this->contact_stiffness);
	symx::Scalar dhat = this->_get_contact_distance(energy, group_a, group_b);
	symx::Scalar E = this->_edge_edge_mollifier(ea, eb, ea_rest, eb_rest) * this->_barrier_potential(d, dhat, k);
	energy.set(E);
}
void EnergyFrictionalContact::_set_friction_potential(symx::Energy& energy, const core::Stark& stark, const symx::Vector& v, const symx::Index& contact_idx, const FrictionContact& contact)
{
	symx::Matrix T = energy.make_matrix(contact.T, { 2, 3 }, contact_idx);
	symx::Scalar mu = energy.make_scalar(contact.mu, contact_idx);
	symx::Scalar fn = energy.make_scalar(contact.fn, contact_idx);
	symx::Scalar epsv = energy.make_scalar(this->global_params.friction_stick_slide_threshold);
	symx::Scalar dt = energy.make_scalar(stark.dt);
	symx::Scalar E = this->_friction_potential(v, fn, mu, T, epsv, dt);
	energy.set(E);
}
void EnergyFrictionalContact::_set_friction_point_edge(symx::Energy& energy, const core::Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& ve, const symx::Index& contact_idx, FrictionPointEdge& data)
{
	symx::Vector bary = energy.make_vector(data.bary, contact_idx);

	symx::Vector va = vp;
	symx::Vector vb = bary[0] * ve[0] + bary[1] * ve[1];
	symx::Vector v = vb - va;
	this->_set_friction_potential(energy, stark, v, contact_idx, data.contact);
}
void EnergyFrictionalContact::_set_friction_point_triangle(symx::Energy& energy, const core::Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& vt, const symx::Index& contact_idx, FrictionPointTriangle& data)
{
	symx::Vector bary = energy.make_vector(data.bary, contact_idx);

	symx::Vector va = vp;
	symx::Vector vb = bary[0] * vt[0] + bary[1] * vt[1] + bary[2] * vt[2];
	symx::Vector v = vb - va;
	this->_set_friction_potential(energy, stark, v, contact_idx, data.contact);
}
void EnergyFrictionalContact::_set_friction_edge_edge(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Vector>& vea, const std::vector<symx::Vector>& veb, const symx::Index& contact_idx, FrictionEdgeEdge& data)
{
	symx::Vector bary = energy.make_vector(data.bary, contact_idx);

	symx::Vector va = vea[0] + bary[0] * (vea[1] - vea[0]);
	symx::Vector vb = veb[0] + bary[1] * (veb[1] - veb[0]);
	symx::Vector v = vb - va;
	this->_set_friction_potential(energy, stark, v, contact_idx, data.contact);
}


/* =================================================================================================== */
/* ===================================  SYMX DATA-SYMBOLS MAPPING  =================================== */
/* =================================================================================================== */
std::vector<symx::Vector> EnergyFrictionalContact::_get_rb_v1(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	symx::Scalar dt = energy.make_scalar(stark.dt);
	std::vector<symx::Vector> x_loc = energy.make_vectors(this->rigidbody_local_vertices.data, conn);
	return this->rb->get_v1(energy, rb_idx, x_loc, dt);
}
std::vector<symx::Vector> EnergyFrictionalContact::_get_rb_x1(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	symx::Scalar dt = energy.make_scalar(stark.dt);
	std::vector<symx::Vector> x_loc = energy.make_vectors(this->rigidbody_local_vertices.data, conn);
	return this->rb->get_x1(energy, rb_idx, x_loc, dt);
}
std::vector<symx::Vector> EnergyFrictionalContact::_get_rb_X(symx::Energy& energy, const std::vector<symx::Index>& conn)
{
	return energy.make_vectors(this->rigidbody_local_vertices.data, conn);
}
std::array<std::vector<symx::Vector>, 2> EnergyFrictionalContact::_get_rb_edge(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"] }
	std::vector<symx::Vector> ea = this->_get_rb_x1(energy, stark, rb_idx, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_rb_X(energy, { conn[0], conn[1] });
	return { ea, ea_rest };
}
std::array<std::vector<symx::Vector>, 3> EnergyFrictionalContact::_get_rb_edge_point(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"], conn["p"] }
	std::vector<symx::Vector> ea = this->_get_rb_x1(energy, stark, rb_idx, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_rb_X(energy, { conn[0], conn[1] });
	std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, rb_idx, { conn[2] });
	return { ea, ea_rest, p };
}
std::vector<symx::Vector> EnergyFrictionalContact::_get_d_v1(symx::Energy& energy, const std::vector<symx::Index>& conn)
{
	return energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, conn);
}
std::vector<symx::Vector> EnergyFrictionalContact::_get_d_x1(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Index>& conn)
{
	std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, conn);
	std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, conn);
	symx::Scalar dt = energy.make_scalar(stark.dt);
	return time_integration(x0, v1, dt);
}
std::vector<symx::Vector> EnergyFrictionalContact::_get_d_X(symx::Energy& energy, const std::vector<symx::Index>& conn)
{
	return energy.make_vectors(this->dyn->X.data, conn);
}
std::array<std::vector<symx::Vector>, 2> EnergyFrictionalContact::_get_d_edge(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"] }
	std::vector<symx::Vector> ea = this->_get_d_x1(energy, stark, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_d_X(energy, { conn[0], conn[1] });
	return { ea, ea_rest };
}
std::array<std::vector<symx::Vector>, 3> EnergyFrictionalContact::_get_d_edge_point(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"], conn["p"] }
	std::vector<symx::Vector> ea = this->_get_d_x1(energy, stark, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_d_X(energy, { conn[0], conn[1] });
	std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn[2] });
	return { ea, ea_rest, p };
}


/* ============================================================================== */
/* ===================================  MISC  =================================== */
/* ============================================================================== */
std::string EnergyFrictionalContact::_get_contact_label(const std::string physical_system, const std::string pair) const
{
	std::string output = "contact_" + physical_system + "_" + pair + "_";
	if (this->ipc_barrier_type == IPCBarrierType::Cubic) {
		output += "cubic";
	}
	else if (this->ipc_barrier_type == IPCBarrierType::Log) {
		output += "log";
	}
	return output;
}
std::string EnergyFrictionalContact::_get_friction_label(const std::string physical_system, const std::string pair) const
{
	std::string output = "friction_" + physical_system + "_" + pair + "_";
	if (this->ipc_friction_type == IPCFrictionType::C0) {
		output += "C0";
	}
	else if (this->ipc_friction_type == IPCFrictionType::C1) {
		output += "C1";
	}
	return output;
}

