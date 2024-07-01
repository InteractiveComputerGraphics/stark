#include "EnergyFrictionalContact.h"

#include "../time_integration.h"
#include "../distances.h"
#include "../rigidbodies/rigidbody_transformations.h"
#include "../../utils/mesh_utils.h"
#include "friction_geometry.h"


stark::EnergyFrictionalContact::EnergyFrictionalContact(core::Stark& stark, const spPointDynamics dyn, const spRigidBodyDynamics rb)
	: dyn(dyn), rb(rb)
{
	if (stark.settings.contact.collisions_enabled) {
		// Check parameters
		if (stark.settings.contact.dhat <= 0.0) {
			std::cout << "stark error: EnergyFrictionalContact::EnergyFrictionalContact() got dhat <= 0.0." << std::endl;
			exit(-1);
		}
		if (stark.settings.contact.adaptive_contact_stiffness.value <= 0.0) {
			std::cout << "stark error: EnergyFrictionalContact::EnergyFrictionalContact() got adaptive_contact_stiffness <= 0.0." << std::endl;
			exit(-1);
		}

		// Callbacks
		stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step__update_friction_contacts(stark); });
		stark.callbacks.before_energy_evaluation.push_back([&]() { this->_before_energy_evaluation__update_contacts(stark); });
		stark.callbacks.is_intermidiate_state_valid.push_back([&]() { return this->_is_valid_configuration(stark); });

		// Energy declarations
		this->_energies_contact_deformables(stark);
		this->_energies_contact_rb(stark);
		this->_energies_contact_rb_deformables(stark);
		this->_energies_contact_static(stark);


		if (stark.settings.contact.friction_enabled) {
			// Check parameters
			if (stark.settings.contact.friction_stick_slide_threshold <= 0.0) {
				std::cout << "stark error: EnergyFrictionalContact::EnergyFrictionalContact() got friction_stick_slide_threshold <= 0.0." << std::endl;
				exit(-1);
			}

			// Callbacks
			stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step__update_friction_contacts(stark); });

			// Energy declarations
			this->_energies_friction_deformables(stark);
			this->_energies_friction_rb(stark);
			this->_energies_friction_rb_deformables(stark);
			this->_energies_friction_static(stark);
		}
	}
}

void stark::EnergyFrictionalContact::set_params(const Params& params)
{
	if (params.stiffness < 0.0) {
		std::cout << "stark error: EnergyFrictionalContact::set_params() got stiffness < 0.0." << std::endl;
		exit(-1);
	}
	this->params = params;
	this->base_stiffness = params.stiffness;
}

stark::EnergyFrictionalContact::Params stark::EnergyFrictionalContact::get_params(const Params& params)
{
	return this->params;
}

int stark::EnergyFrictionalContact::add_rigid_body(const int idx, const std::vector<std::array<int, 2>>& edges, const std::vector<Eigen::Vector3d>& vertices)
{
	if (this->rigidbody_local_vertices.get_n_sets() != idx) {
		std::cout << "stark error: Rigid bodies must be passed in creation order to EnergyFrictionalContact." << std::endl;
		exit(-1);
	}
	this->rigidbody_local_vertices.append(vertices);
	const int mesh_idx = this->_add_edges_and_points(PhysicalSystem::Rigidbody, idx, edges, (int)vertices.size());
	this->disable_collision(mesh_idx, mesh_idx);
	this->rigidbody_idx_collision_idx_map[idx] = mesh_idx;
	return mesh_idx;
}
int stark::EnergyFrictionalContact::add_rigid_body(const int idx, const std::vector<std::array<int, 3>>& triangles, const std::vector<Eigen::Vector3d>& vertices)
{
	if (this->rigidbody_local_vertices.get_n_sets() != idx) {
		std::cout << "stark error: Rigid bodies must be passed in creation order to EnergyFrictionalContact." << std::endl;
		exit(-1);
	}
	this->rigidbody_local_vertices.append(vertices);
	const int mesh_idx = this->_add_triangles_edges_and_points(PhysicalSystem::Rigidbody, idx, triangles, (int)vertices.size());
	this->disable_collision(mesh_idx, mesh_idx);
	this->rigidbody_idx_collision_idx_map[idx] = mesh_idx;
	return mesh_idx;
}
int stark::EnergyFrictionalContact::add_deformable(const int idx, const std::vector<std::array<int, 3>>& triangles, const std::vector<int>& surface_node_map)
{
	if ((int)this->surface_node_maps.size() != idx) {
		std::cout << "stark error: Deformables must be passed in creation order to EnergyFrictionalContact." << std::endl;
		exit(-1);
	}
	this->surface_node_maps.push_back(surface_node_map); // Keep the map from all the vertices of the simulation mesh to the contact mesh (e.g. tetrahedral meshes)
	const int mesh_idx = this->_add_triangles_edges_and_points(PhysicalSystem::Deformable, idx, triangles, (int)surface_node_map.size());
	this->deformable_idx_collision_idx_map[idx] = mesh_idx;
	return mesh_idx;
}
int stark::EnergyFrictionalContact::add_deformable(const int idx, const std::vector<std::array<int, 3>>& triangles, const int n_points)
{
	if ((int)this->surface_node_maps.size() != idx) {
		std::cout << "stark error: Deformables must be passed in creation order to EnergyFrictionalContact." << std::endl;
		exit(-1);
	}
	this->surface_node_maps.push_back({});  // All nodes are in the contact mesh
	const int mesh_idx = this->_add_triangles_edges_and_points(PhysicalSystem::Deformable, idx, triangles, n_points);
	this->deformable_idx_collision_idx_map[idx] = mesh_idx;
	return mesh_idx;
}
int stark::EnergyFrictionalContact::add_deformable(const int idx, const std::vector<std::array<int, 2>>& edges, const int n_points)
{
	if ((int)this->surface_node_maps.size() != idx) {
		std::cout << "stark error: Deformables must be passed in creation order to EnergyFrictionalContact." << std::endl;
		exit(-1);
	}
	this->surface_node_maps.push_back({});  // All nodes are in the contact mesh
	const int mesh_idx = this->_add_edges_and_points(PhysicalSystem::Deformable, idx, edges, n_points);
	this->deformable_idx_collision_idx_map[idx] = mesh_idx;
	return mesh_idx;
}
bool stark::EnergyFrictionalContact::is_deformable_declared(const int idx) const
{
	return this->surface_node_maps.size() > idx;
}
int stark::EnergyFrictionalContact::add_static_plane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal)
{
	this->static_planes.point.push_back(point);
	this->static_planes.normal.push_back(normal);
	return (int)this->static_planes.point.size() - 1;
}
void stark::EnergyFrictionalContact::set_coulomb_friction_pair(const PhysicalSystem& ps0, const int idx0_in_ps, const PhysicalSystem& ps1, const int idx1_in_ps, const double mu)
{
	this->set_coulomb_friction_pair(this->_get_collision_idx(ps0, idx0_in_ps), this->_get_collision_idx(ps1, idx1_in_ps), mu);
}
void stark::EnergyFrictionalContact::set_coulomb_friction_pair_static(const int static_idx, const PhysicalSystem& ps, const int idx_in_ps, const double mu)
{
	const std::array<int, 2> pair = { static_idx, this->_get_collision_idx(ps, idx_in_ps) };
	this->static_pair_coulombs_mu[pair] = mu;
}
void stark::EnergyFrictionalContact::set_coulomb_friction_pair(const int idx0, const int idx1, const double mu)
{
	const std::array<int, 2> pair = { std::min(idx0, idx1), std::max(idx0, idx1) };
	this->pair_coulombs_mu[pair] = mu;
}
void stark::EnergyFrictionalContact::disable_collision(const int idx0, const int idx1)
{
	const std::array<int, 2> pair = { std::min(idx0, idx1), std::max(idx0, idx1) };
	if (this->disabled_collision_pairs.find(pair) == this->disabled_collision_pairs.end()) {
		this->disabled_collision_pairs.insert(pair);
		this->pd.add_blacklist(pair[0], pair[1]);
		this->id.add_blacklist(pair[0], pair[1]);
	}
}
void stark::EnergyFrictionalContact::disable_collision(const PhysicalSystem& ps0, const int idx0_in_ps, const PhysicalSystem& ps1, const int idx1_in_ps)
{
	this->disable_collision(this->_get_collision_idx(ps0, idx0_in_ps), this->_get_collision_idx(ps1, idx1_in_ps));
}
void stark::EnergyFrictionalContact::disable_collision_static(int static_idx, const PhysicalSystem& ps, const int idx_in_ps)
{
	const std::array<int, 2> pair = { static_idx, this->_get_collision_idx(ps, idx_in_ps) };
	if (this->static_disabled_collision_pairs.find(pair) == this->static_disabled_collision_pairs.end()) {
		this->static_disabled_collision_pairs.insert(pair);
	}

}

/* ================================================================================= */
/* ===================================  HELPERS  =================================== */
/* ================================================================================= */
int stark::EnergyFrictionalContact::_get_collision_idx(const PhysicalSystem& ps, const int idx_in_ps)
{
	if (ps == PhysicalSystem::Deformable) {
		if (this->deformable_idx_collision_idx_map.find(idx_in_ps) == this->deformable_idx_collision_idx_map.end()) {
			std::cout << "stark error: Deformable idx not found in EnergyFrictionalContact::disable_collision()." << std::endl;
			exit(-1);
		}
		return this->deformable_idx_collision_idx_map[idx_in_ps];
	}
	else if (ps == PhysicalSystem::Rigidbody) {
		if (this->rigidbody_idx_collision_idx_map.find(idx_in_ps) == this->rigidbody_idx_collision_idx_map.end()) {
			std::cout << "stark error: Rigidbody idx not found in EnergyFrictionalContact::disable_collision()." << std::endl;
			exit(-1);
		}
		return this->rigidbody_idx_collision_idx_map[idx_in_ps];
	}
	else {
		std::cout << "stark error: Unknown physical system found in EnergyFrictionalContact::disable_collision()." << std::endl;
		exit(-1);
	}
}
int stark::EnergyFrictionalContact::_add_edges_and_points(const PhysicalSystem& ps, const int idx, const std::vector<std::array<int, 2>>& edges, const int n_points)
{
	// Add to local meshes data structures
	this->meshes.push_back({});
	auto& mesh = this->meshes.back();
	mesh.ps = ps;
	mesh.ps_set = idx;
	mesh.vertices.resize(n_points, Eigen::Vector3d::Zero());
	mesh.edges.insert(mesh.edges.end(), edges.begin(), edges.end());
	mesh.triangles.push_back({});

	// Add to collision detection
	this->pd.add_mesh(&mesh.vertices[0][0], n_points, &mesh.triangles[0][0], (int)mesh.triangles.size(), &mesh.edges[0][0], (int)mesh.edges.size());
	this->id.add_mesh(&mesh.vertices[0][0], n_points, &mesh.triangles[0][0], (int)mesh.triangles.size(), &mesh.edges[0][0], (int)mesh.edges.size());

	return (int)this->meshes.size() - 1;
}
int stark::EnergyFrictionalContact::_add_triangles_edges_and_points(const PhysicalSystem& ps, const int idx, const std::vector<std::array<int, 3>>& triangles, const int n_points)
{
	// Find edges
	std::vector<std::array<int, 2>> edges;
	utils::find_edges_from_simplices(edges, triangles, n_points);

	// Add to local meshes data structures
	this->meshes.push_back({});
	auto& mesh = this->meshes.back();
	mesh.ps = ps;
	mesh.ps_set = idx;
	mesh.vertices.resize(n_points, Eigen::Vector3d::Zero());
	mesh.edges.insert(mesh.edges.end(), edges.begin(), edges.end());
	mesh.triangles.insert(mesh.triangles.end(), triangles.begin(), triangles.end());

	// Add to collision detection
	this->pd.add_mesh(&mesh.vertices[0][0], n_points, &mesh.triangles[0][0], (int)mesh.triangles.size(), &mesh.edges[0][0], (int)mesh.edges.size());
	this->id.add_mesh(&mesh.vertices[0][0], n_points, &mesh.triangles[0][0], (int)mesh.triangles.size(), &mesh.edges[0][0], (int)mesh.edges.size());

	return (int)this->meshes.size() - 1;
}
void stark::EnergyFrictionalContact::_update_vertices(core::Stark& stark, const double dt)
{
	#pragma omp parallel for schedule(static) num_threads(stark.settings.execution.n_threads)
	for (int local_idx = 0; local_idx < (int)this->meshes.size(); local_idx++) {
		auto& mesh = this->meshes[local_idx];
		const int n = (int)mesh.vertices.size();

		// Physical System
		if (mesh.ps == PhysicalSystem::Deformable) {
			const int ps_begin = this->dyn->x0.get_begin(mesh.ps_set);
			const std::vector<int>& surface_node_map = this->surface_node_maps[mesh.ps_set];
			if (surface_node_map.size() == 0) {  // All nodes are surface nodes
				for (int i = 0; i < n; i++) {
					mesh.vertices[i] = time_integration(this->dyn->x0[ps_begin + i], this->dyn->v1[ps_begin + i], dt);
				}
			}
			else {
				for (int surf_i = 0; surf_i < (int)surface_node_map.size(); surf_i++) {  // Use only the surface nodes
					const int i = surface_node_map[surf_i];
					mesh.vertices[surf_i] = time_integration(this->dyn->x0[ps_begin + i], this->dyn->v1[ps_begin + i], dt);
				}
			}
		}
		else if (mesh.ps == PhysicalSystem::Rigidbody) {
			const int ps_begin = this->rigidbody_local_vertices.get_begin(mesh.ps_set);
			const int ps_end = this->rigidbody_local_vertices.get_end(mesh.ps_set);
			const int ps_n = ps_end - ps_begin;

			if (ps_n != n) {
				stark.console.print("stark error: Number of vertices mismatch found in EnergyFrictionalContact (rigidbodies).\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}

			// Calculate the transformation using Newton's proposed v1.
			const Eigen::Vector3d t1 = time_integration(this->rb->t0[mesh.ps_set], this->rb->v1[mesh.ps_set], dt);
			const Eigen::Quaterniond q1 = quat_time_integration(this->rb->q0[mesh.ps_set], this->rb->w1[mesh.ps_set], dt);
			const Eigen::Matrix3d R1 = q1.toRotationMatrix();
			for (int i = 0; i < n; i++) {
				mesh.vertices[i] = local_to_global_point(this->rigidbody_local_vertices[ps_begin + i], R1, t1);
			}
		}
		else {
			stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
			exit(-1);
		}
	}
}
void stark::EnergyFrictionalContact::_update_contacts_static(core::Stark& stark)
{
	// Note: Assumes vertices are already updated from _update_vertices used previously for deformables and rigid bodies
	this->contacts_static.clear();

	// Planes
	const int n_planes = (int)this->static_planes.point.size();
	const double dhat2 = stark.settings.contact.dhat * stark.settings.contact.dhat;
	for (int mesh_i = 0; mesh_i < (int)this->meshes.size(); mesh_i++) {
		const Mesh& mesh = this->meshes[mesh_i];
		for (int plane_i = 0; plane_i < n_planes; plane_i++) {
			if (this->_is_disabled_collision_pair_static(plane_i, mesh_i)) { continue; }
			for (int loc = 0; loc < (int)mesh.vertices.size(); loc++) {
				if (sq_distance_point_plane(mesh.vertices[loc], this->static_planes.point[plane_i], this->static_planes.normal[plane_i]) < dhat2) {
					
					if (mesh.ps == PhysicalSystem::Deformable) {
						const int glob = this->_local_to_ps_global_indices<1>(PhysicalSystem::Deformable, mesh.ps_set, { loc })[0];  // Note: Takes care of the surface node map
						this->contacts_static.deformable_point.push_back({ plane_i, glob });
					}
					else if (mesh.ps == PhysicalSystem::Rigidbody) {
						const int glob = this->_local_to_ps_global_indices<1>(PhysicalSystem::Rigidbody, mesh.ps_set, { loc })[0];
						this->contacts_static.rb_point.push_back({ plane_i, mesh.ps_set, glob });
					}
					else {
						stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
						exit(-1);
					}
				}
			}
		}
	}
}
bool stark::EnergyFrictionalContact::_intersection_static(core::Stark& stark)
{
	// Note: Assumes vertices are already updated from _update_vertices used previously for deformables and rigid bodies
	bool intersections_found = false;

	// Planes
	const int n_planes = (int)this->static_planes.point.size();
	for (int mesh_i = 0; mesh_i < (int)this->meshes.size(); mesh_i++) {
		const Mesh& mesh = this->meshes[mesh_i];
		for (int plane_i = 0; plane_i < n_planes; plane_i++) {
			if (this->_is_disabled_collision_pair_static(plane_i, mesh_i)) { continue; }
			for (int loc = 0; loc < (int)mesh.vertices.size(); loc++) {
				if (signed_distance_point_plane(mesh.vertices[loc], this->static_planes.point[plane_i], this->static_planes.normal[plane_i]) < 0.0) {
					intersections_found = true;
				}
			}
		}
	}

	return intersections_found;
}
void stark::EnergyFrictionalContact::_update_friction_static(core::Stark& stark)
{
	// Note: Assumes vertices are already updated from _update_vertices used previously for deformables and rigid bodies (dt = 0.0)
	this->friction_static.clear();
	const int n_planes = (int)this->static_planes.point.size();

	// Precompute projection matrices for planes
	std::vector<std::array<double, 6>> plane_projection_matrices(n_planes);
	for (int plane_i = 0; plane_i < n_planes; plane_i++) {
		plane_projection_matrices[plane_i] = projection_matrix_point_point(this->static_planes.point[plane_i] + this->static_planes.normal[plane_i], this->static_planes.point[plane_i]);
	}

	// Planes
	const double dhat2 = stark.settings.contact.dhat * stark.settings.contact.dhat;
	for (int mesh_i = 0; mesh_i < (int)this->meshes.size(); mesh_i++) {
		const Mesh& mesh = this->meshes[mesh_i];
		for (int plane_i = 0; plane_i < n_planes; plane_i++) {
			const double mu = this->_get_friction_static(plane_i, mesh_i);
			if (mu == 0.0) { continue; }
			for (int loc = 0; loc < (int)mesh.vertices.size(); loc++) {
				const double d2 = sq_distance_point_plane(mesh.vertices[loc], this->static_planes.point[plane_i], this->static_planes.normal[plane_i]);
				if (d2 < dhat2) {
					const double d = std::sqrt(d2);
					const double k = stark.settings.contact.adaptive_contact_stiffness.value;
					const double dhat = stark.settings.contact.dhat;
					const double force = this->_barrier_force(d, dhat, k);

					if (mesh.ps == PhysicalSystem::Deformable) {
						this->friction_static.deformable_point.contact.T.push_back(plane_projection_matrices[plane_i]);
						this->friction_static.deformable_point.contact.mu.push_back(mu);
						this->friction_static.deformable_point.contact.fn.push_back(force);
						const int glob = this->_local_to_ps_global_indices<1>(PhysicalSystem::Deformable, mesh.ps_set, { loc })[0];  // Note: Takes care of the surface node map
						this->friction_static.deformable_point.conn.numbered_push_back({ glob });
					}
					else if (mesh.ps == PhysicalSystem::Rigidbody) {
						this->friction_static.rb_point.contact.T.push_back(plane_projection_matrices[plane_i]);
						this->friction_static.rb_point.contact.mu.push_back(mu);
						this->friction_static.rb_point.contact.fn.push_back(force);
						const int glob = this->_local_to_ps_global_indices<1>(PhysicalSystem::Rigidbody, mesh.ps_set, { loc })[0];
						this->friction_static.rb_point.conn.numbered_push_back({ mesh.ps_set, glob });
					}
					else {
						stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
						exit(-1);
					}
				}
			}
		}
	}
}
const tmcd::ProximityResults& stark::EnergyFrictionalContact::_run_proximity_detection(core::Stark& stark, const double dt)
{
	this->_update_vertices(stark, dt);

	this->pd.set_n_threads(stark.settings.execution.n_threads);
	this->pd.set_edge_edge_parallel_cutoff(stark.settings.contact.edge_edge_cross_norm_sq_cutoff);
	this->pd.activate_point_triangle(stark.settings.contact.triangle_point_enabled);
	this->pd.activate_edge_edge(stark.settings.contact.edge_edge_enabled);
	return this->pd.run(stark.settings.contact.dhat);
}
const tmcd::IntersectionResults& stark::EnergyFrictionalContact::_run_intersection_detection(core::Stark& stark, const double dt)
{
	this->_update_vertices(stark, dt);

	this->id.set_n_threads(stark.settings.execution.n_threads);
	return this->id.run();
}

/* ============================================================================================= */
/* ===================================  COLLISION DETECTION  =================================== */
/* ============================================================================================= */
template<std::size_t N>
std::array<int, N> stark::EnergyFrictionalContact::_local_to_ps_global_indices(const PhysicalSystem& ps, const int set_idx, const std::array<int, N>& local)
{
	std::array<int, N> global;
	switch (ps)
	{
	case PhysicalSystem::Deformable:
		if (this->surface_node_maps[set_idx].size() == 0) {  // All nodes are surface nodes
			for (int i = 0; i < N; i++) {
				global[i] = this->dyn->x1.get_global_index(set_idx, local[i]);
			}
		}
		else {
			for (int i = 0; i < N; i++) {
				global[i] = this->dyn->x1.get_global_index(set_idx, this->surface_node_maps[set_idx][local[i]]);
			}
		}
		break;
	case PhysicalSystem::Rigidbody:
		for (int i = 0; i < N; i++) {
			global[i] = this->rigidbody_local_vertices.get_global_index(set_idx, local[i]);
		}
		break;
	default:
		break;
	}
	return global;
}
template std::array<int, 1> stark::EnergyFrictionalContact::_local_to_ps_global_indices<1>(const PhysicalSystem& ps, const int set_idx, const std::array<int, 1>& local);
template std::array<int, 2> stark::EnergyFrictionalContact::_local_to_ps_global_indices<2>(const PhysicalSystem& ps, const int set_idx, const std::array<int, 2>& local);
template std::array<int, 3> stark::EnergyFrictionalContact::_local_to_ps_global_indices<3>(const PhysicalSystem& ps, const int set_idx, const std::array<int, 3>& local);
stark::ProximityHelper<1> stark::EnergyFrictionalContact::_get_proximity_helper_point(const tmcd::Point& point)
{
	ProximityHelper<1> helper;
	helper.set = point.set;
	helper.ps = this->meshes[point.set].ps;
	helper.ps_set = this->meshes[point.set].ps_set;
	helper.local_verts = { point.idx };
	helper.verts = this->_local_to_ps_global_indices<1>(helper.ps, helper.ps_set, helper.local_verts);
	return helper;
}
stark::ProximityHelper<2> stark::EnergyFrictionalContact::_get_proximity_helper_edge(const tmcd::TriangleEdge::Edge& edge)
{
	ProximityHelper<2> helper;
	helper.set = edge.set;
	helper.ps = this->meshes[edge.set].ps;
	helper.ps_set = this->meshes[edge.set].ps_set;
	helper.local_verts = edge.vertices;
	helper.verts = this->_local_to_ps_global_indices(helper.ps, helper.ps_set, helper.local_verts);
	return helper;
}
stark::ProximityHelper<3> stark::EnergyFrictionalContact::_get_proximity_helper_triangle(const tmcd::Triangle& triangle)
{
	ProximityHelper<3> helper;
	helper.set = triangle.set;
	helper.ps = this->meshes[triangle.set].ps;
	helper.ps_set = this->meshes[triangle.set].ps_set;
	helper.local_verts = triangle.vertices;
	helper.verts = this->_local_to_ps_global_indices(helper.ps, helper.ps_set, helper.local_verts);
	return helper;
}
stark::ProximityHelper<1> stark::EnergyFrictionalContact::_get_proximity_helper_edge_point(const tmcd::EdgePoint& edge_point)
{
	ProximityHelper<1> helper = this->_get_proximity_helper_point(edge_point.point);
	helper.edge = this->_local_to_ps_global_indices(helper.ps, helper.ps_set, edge_point.edge.vertices);
	return helper;
}
stark::ProximityHelper<2> stark::EnergyFrictionalContact::_get_proximity_helper_edge(const tmcd::Edge& edge)
{
	ProximityHelper<2> helper;
	helper.set = edge.set;
	helper.ps = this->meshes[edge.set].ps;
	helper.ps_set = this->meshes[edge.set].ps_set;
	helper.local_verts = edge.vertices;
	helper.verts = this->_local_to_ps_global_indices(helper.ps, helper.ps_set, helper.local_verts);
	helper.edge = helper.verts;
	return helper;
}
double stark::EnergyFrictionalContact::_get_friction(const int idx0, const int idx1)
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
double stark::EnergyFrictionalContact::_get_friction_static(const int static_idx, const int object_idx)
{
	const std::array<int, 2> pair = { static_idx, object_idx };
	auto it = this->static_pair_coulombs_mu.find(pair);
	if (it == this->static_pair_coulombs_mu.end()) {
		return 0.0;
	}
	else {
		return it->second;
	}
}
bool stark::EnergyFrictionalContact::_is_disabled_collision_pair_static(const int static_idx, const int object_idx)
{
	const std::array<int, 2> pair = { static_idx, object_idx };
	return this->static_disabled_collision_pairs.find(pair) != this->static_disabled_collision_pairs.end();
}


/* ======================================================================================== */
/* ===================================  SYMX CALLBACKS  =================================== */
/* ======================================================================================== */
void stark::EnergyFrictionalContact::_before_energy_evaluation__update_contacts(core::Stark& stark)
{
	if (!stark.settings.contact.collisions_enabled) { return; }

	// Clear contact connectivity
	this->contacts_deformables.clear();
	this->contacts_rb.clear();
	this->contacts_rb_deformables.clear();

	// Run proximity detection
	const auto& proximity = this->_run_proximity_detection(stark, stark.settings.simulation.adaptive_time_step.value);

	// Solve static objects (vertices are already updated from `this->_run_proximity_detection`)
	this->_update_contacts_static(stark);

	// Point - Triangle
	{
		// Point - Triangle
		for (const auto& pair : proximity.point_triangle.point_point) {
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<1> B = this->_get_proximity_helper_point(pair.second.point);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.point_triangle.point_point.push_back({ A.verts[0], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.point_triangle.point_point.push_back({ A.ps_set, B.ps_set, A.verts[0], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.point_triangle.rb_d_point_point.push_back({ A.ps_set, A.verts[0], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.point_triangle.rb_d_point_point.push_back({ B.ps_set, B.verts[0], A.verts[0] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Edge
		for (const auto& pair : proximity.point_triangle.point_edge) {
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second.edge);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.point_triangle.point_edge.push_back({ A.verts[0], B.verts[0], B.verts[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.point_triangle.point_edge.push_back({ A.ps_set, B.ps_set, A.verts[0], B.verts[0], B.verts[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.point_triangle.rb_d_point_edge.push_back({ A.ps_set, A.verts[0], B.verts[0], B.verts[1] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.point_triangle.rb_d_edge_point.push_back({ B.ps_set, B.verts[0], B.verts[1], A.verts[0] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Triangle
		for (const auto& pair : proximity.point_triangle.point_triangle) {
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<3> B = this->_get_proximity_helper_triangle(pair.second);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.point_triangle.point_triangle.push_back({ A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.point_triangle.point_triangle.push_back({ A.ps_set, B.ps_set, A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.point_triangle.rb_d_point_triangle.push_back({ A.ps_set, A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.point_triangle.rb_d_triangle_point.push_back({ B.ps_set, B.verts[0], B.verts[1], B.verts[2], A.verts[0] });
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
			const ProximityHelper<1> A = this->_get_proximity_helper_edge_point(pair.first);
			const ProximityHelper<1> B = this->_get_proximity_helper_edge_point(pair.second);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.edge_edge.point_point.push_back({ A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.edge_edge.point_point.push_back({ A.ps_set, B.ps_set, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.edge_edge.rb_d_point_point.push_back({ A.ps_set, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1], B.verts[0] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.edge_edge.rb_d_point_point.push_back({ B.ps_set, B.edge[0], B.edge[1], B.verts[0], A.edge[0], A.edge[1], A.verts[0] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Edge
		for (const auto& pair : proximity.edge_edge.point_edge) {
			const ProximityHelper<1> A = this->_get_proximity_helper_edge_point(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.edge_edge.point_edge.push_back({ A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.edge_edge.point_edge.push_back({ A.ps_set, B.ps_set, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.edge_edge.rb_d_point_edge.push_back({ A.ps_set, A.edge[0], A.edge[1], A.verts[0], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.edge_edge.rb_d_edge_point.push_back({ B.ps_set, B.edge[0], B.edge[1], A.edge[0], A.edge[1], A.verts[0] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Edge - Edge
		for (const auto& pair : proximity.edge_edge.edge_edge) {
			const ProximityHelper<2> A = this->_get_proximity_helper_edge(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second);

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->contacts_deformables.edge_edge.edge_edge.push_back({ A.edge[0], A.edge[1], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb.edge_edge.edge_edge.push_back({ A.ps_set, B.ps_set, A.edge[0], A.edge[1], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->contacts_rb_deformables.edge_edge.rb_d_edge_edge.push_back({ A.ps_set, A.edge[0], A.edge[1], B.edge[0], B.edge[1] });
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->contacts_rb_deformables.edge_edge.rb_d_edge_edge.push_back({ B.ps_set, B.edge[0], B.edge[1], A.edge[0], A.edge[1] });
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}
	} // Edge - Edge
}
void stark::EnergyFrictionalContact::_before_time_step__update_friction_contacts(core::Stark& stark)
{
	if (!stark.settings.contact.collisions_enabled) { return; }
	if (!stark.settings.contact.friction_enabled) { return; }

	// Clear contact connectivity
	this->friction_deformables.clear();
	this->friction_rb.clear();
	this->friction_rb_deformables.clear();

	// Run proximity detection
	const auto& proximity = this->_run_proximity_detection(stark, /* dt = */ 0.0);

	// Solve static objects
	this->_update_friction_static(stark);

	// Lambdas
	const double k = stark.settings.contact.adaptive_contact_stiffness.value;
	const double dhat = stark.settings.contact.dhat;
	auto force = [&](double d) { return this->_barrier_force(d, dhat, k); };
	auto point_point = [&](FrictionContact& contact, const double mu, const double d, const ProximityHelper<1>& A, const ProximityHelper<1>& B)
	{
		const Eigen::Vector3d& P = this->meshes[A.set].vertices[A.local_verts[0]];
		const Eigen::Vector3d& Q = this->meshes[B.set].vertices[B.local_verts[0]];
		contact.T.push_back(projection_matrix_point_point(P, Q));
		contact.mu.push_back(mu);
		contact.fn.push_back(force(d));
	};
	auto point_edge = [&](FrictionPointEdge& friction, const double mu, const double d, const ProximityHelper<1>& A, const ProximityHelper<2>& B)
	{
		const Eigen::Vector3d& P = this->meshes[A.set].vertices[A.local_verts[0]];
		const Eigen::Vector3d& E0 = this->meshes[B.set].vertices[B.local_verts[0]];
		const Eigen::Vector3d& E1 = this->meshes[B.set].vertices[B.local_verts[1]];
		friction.bary.push_back(barycentric_point_edge(P, E0, E1));
		friction.contact.T.push_back(projection_matrix_point_edge(P, E0, E1));
		friction.contact.mu.push_back(mu);
		friction.contact.fn.push_back(force(d));
	};
	auto point_triangle = [&](FrictionPointTriangle& friction, const double mu, const double d, const ProximityHelper<1>& A, const ProximityHelper<3>& B)
	{
		const Eigen::Vector3d& P  = this->meshes[A.set].vertices[A.local_verts[0]]; 
		const Eigen::Vector3d& T0 = this->meshes[B.set].vertices[B.local_verts[0]]; 
		const Eigen::Vector3d& T1 = this->meshes[B.set].vertices[B.local_verts[1]];
		const Eigen::Vector3d& T2 = this->meshes[B.set].vertices[B.local_verts[2]];
		friction.bary.push_back(barycentric_point_triangle(P, T0, T1, T2));
		friction.contact.T.push_back(projection_matrix_triangle(T0, T1, T2));
		friction.contact.mu.push_back(mu);
		friction.contact.fn.push_back(force(d));
	};

	// Point - Triangle
	{
		//// Point - Point
		for (const auto& pair : proximity.point_triangle.point_point) {
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<1> B = this->_get_proximity_helper_point(pair.second.point);
			const double mu = this->_get_friction(A.set, B.set);
			if (mu == 0.0) { continue; }

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->friction_deformables.point_point.conn.numbered_push_back({ A.verts[0], B.verts[0] });
				point_point(this->friction_deformables.point_point.contact, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb.point_point.conn.numbered_push_back({ A.ps_set, B.ps_set, A.verts[0], B.verts[0] });
				point_point(this->friction_rb.point_point.contact, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->friction_rb_deformables.point_point.conn.numbered_push_back({ A.ps_set, A.verts[0], B.verts[0] });
				point_point(this->friction_rb_deformables.point_point.contact, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb_deformables.point_point.conn.numbered_push_back({ B.ps_set, B.verts[0], A.verts[0] });
				point_point(this->friction_rb_deformables.point_point.contact, mu, pair.distance, A, B);
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Edge
		for (const auto& pair : proximity.point_triangle.point_edge) {
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<2> B = this->_get_proximity_helper_edge(pair.second.edge);
			const double mu = this->_get_friction(A.set, B.set);
			if (mu == 0.0) { continue; }

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->friction_deformables.point_edge.conn.numbered_push_back({ A.verts[0], B.verts[0], B.verts[1] });
				point_edge(this->friction_deformables.point_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb.point_edge.conn.numbered_push_back({ A.ps_set, B.ps_set, A.verts[0], B.verts[0], B.verts[1] });
				point_edge(this->friction_rb.point_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->friction_rb_deformables.point_edge.conn.numbered_push_back({ A.ps_set, A.verts[0], B.verts[0], B.verts[1] });
				point_edge(this->friction_rb_deformables.point_edge.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb_deformables.edge_point.conn.numbered_push_back({ B.ps_set, B.verts[0], B.verts[1], A.verts[0] });
				point_edge(this->friction_rb_deformables.edge_point.data, mu, pair.distance, A, B);
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}

		//// Point - Triangle
		for (const auto& pair : proximity.point_triangle.point_triangle) {
			const ProximityHelper<1> A = this->_get_proximity_helper_point(pair.first);
			const ProximityHelper<3> B = this->_get_proximity_helper_triangle(pair.second);
			const double mu = this->_get_friction(A.set, B.set);
			if (mu == 0.0) { continue; }

			if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Deformable) {
				this->friction_deformables.point_triangle.conn.numbered_push_back({ A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
				point_triangle(this->friction_deformables.point_triangle.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb.point_triangle.conn.numbered_push_back({ A.ps_set, B.ps_set, A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
				point_triangle(this->friction_rb.point_triangle.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Rigidbody && B.ps == PhysicalSystem::Deformable) {
				this->friction_rb_deformables.point_triangle.conn.numbered_push_back({ A.ps_set, A.verts[0], B.verts[0], B.verts[1], B.verts[2] });
				point_triangle(this->friction_rb_deformables.point_triangle.data, mu, pair.distance, A, B);
			}
			else if (A.ps == PhysicalSystem::Deformable && B.ps == PhysicalSystem::Rigidbody) {
				this->friction_rb_deformables.triangle_point.conn.numbered_push_back({ B.ps_set, B.verts[0], B.verts[1], B.verts[2], A.verts[0] });
				point_triangle(this->friction_rb_deformables.triangle_point.data, mu, pair.distance, A, B);
			}
			else {
				stark.console.print("stark error: Unknown physical system found in EnergyFrictionalContact.\n", core::ConsoleVerbosity::Frames);
				exit(-1);
			}
		}
	} // Point - Triangle
}
bool stark::EnergyFrictionalContact::_is_valid_configuration(core::Stark& stark)
{
	if (!stark.settings.contact.collisions_enabled) { return true; }
	if (!stark.settings.contact.enable_intersection_test) { return true; }

	const auto& intersections = this->_run_intersection_detection(stark, stark.settings.simulation.adaptive_time_step.value);
	bool intersections_found = intersections.edge_triangle.size() > 0;
	
	// Static objects
	if (!intersections_found) {
		intersections_found = this->_intersection_static(stark);
	}

	return !intersections_found;
}


/* ========================================================================================== */
/* ===================================  SYMX DEFINITIONS  =================================== */
/* ========================================================================================== */
void stark::EnergyFrictionalContact::_energies_contact_deformables(core::Stark& stark)
{
	// Point - Triangle
	//// Point - Point
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "pt_pp"), this->contacts_deformables.point_triangle.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			std::vector<symx::Vector> q = this->_get_d_x1(energy, stark, { conn["q"]});
			symx::Scalar d = distance_point_point(p[0], q[0]);
			this->_set_barrier_potential(energy, stark, d);
		}
	);

	//// Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "pt_pe"), this->contacts_deformables.point_triangle.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			std::vector<symx::Vector> e = this->_get_d_x1(energy, stark, { conn["e0"], conn["e1"] });
			symx::Scalar d = distance_point_line(p[0], e[0], e[1]);
			this->_set_barrier_potential(energy, stark, d);
		}
	);

	//// Point - Triangle
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "pt_pt"), this->contacts_deformables.point_triangle.point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			std::vector<symx::Vector> t = this->_get_d_x1(energy, stark, { conn["t0"], conn["t1"], conn["t2"] });
			symx::Scalar d = distance_point_plane(p[0], t[0], t[1], t[2]);
			this->_set_barrier_potential(energy, stark, d);
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
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
		}
	);

	//// Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "ee_pe"), this->contacts_deformables.edge_edge.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto [ea, ea_rest, p] = this->_get_d_edge_point(energy, stark, { conn["ea0"], conn["ea1"], conn["p"] });
			auto [eb, eb_rest] = this->_get_d_edge(energy, stark, { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_point_line(p[0], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
		}
	);

	//// Edge - Edge
	stark.global_energy.add_energy(this->_get_contact_label("d_d", "ee_ee"), this->contacts_deformables.edge_edge.edge_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto [ea, ea_rest] = this->_get_d_edge(energy, stark, { conn["ea0"], conn["ea1"] });
			auto [eb, eb_rest] = this->_get_d_edge(energy, stark, { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_line_line(ea[0], ea[1], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
		}
	);
}
void stark::EnergyFrictionalContact::_energies_contact_rb(core::Stark& stark)
{
	// Point - Triangle
	//// Point - Point
	stark.global_energy.add_energy(this->_get_contact_label("rb_rb", "pt_pp"), this->contacts_rb.point_triangle.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["a"], { conn["p"] });
			std::vector<symx::Vector> q = this->_get_rb_x1(energy, stark, conn["b"], { conn["q"] });
			symx::Scalar d = distance_point_point(p[0], q[0]);
			this->_set_barrier_potential(energy, stark, d);
		}
	);

	//// Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_rb", "pt_pe"), this->contacts_rb.point_triangle.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["a"], { conn["p"] });
			std::vector<symx::Vector> e = this->_get_rb_x1(energy, stark, conn["b"], { conn["e0"], conn["e1"] });
			symx::Scalar d = distance_point_line(p[0], e[0], e[1]);
			this->_set_barrier_potential(energy, stark, d);
		}
	);

	//// Point - Triangle
	stark.global_energy.add_energy(this->_get_contact_label("rb_rb", "pt_pt"), this->contacts_rb.point_triangle.point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["a"], { conn["p"] });
			std::vector<symx::Vector> t = this->_get_rb_x1(energy, stark, conn["b"], { conn["t0"], conn["t1"], conn["t2"] });
			symx::Scalar d = distance_point_plane(p[0], t[0], t[1], t[2]);
			this->_set_barrier_potential(energy, stark, d);
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
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
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
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
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
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
		}
	);
}
void stark::EnergyFrictionalContact::_energies_contact_rb_deformables(core::Stark& stark)
{
	// Point - Triangle
	//// RB -> D: Point - Point
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_pp"), this->contacts_rb_deformables.point_triangle.rb_d_point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["rb"], { conn["p"] });
			std::vector<symx::Vector> q = this->_get_d_x1(energy, stark, { conn["q"] });
			symx::Scalar d = distance_point_point(p[0], q[0]);
			this->_set_barrier_potential(energy, stark, d);
		}
	);

	//// RB -> D: Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_pe"), this->contacts_rb_deformables.point_triangle.rb_d_point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["rb"], { conn["p"] });
			std::vector<symx::Vector> e = this->_get_d_x1(energy, stark, { conn["e0"], conn["e1"]});
			symx::Scalar d = distance_point_line(p[0], e[0], e[1]);
			this->_set_barrier_potential(energy, stark, d);
		}
	);

	//// RB -> D: Point - Triangle
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_pt"), this->contacts_rb_deformables.point_triangle.rb_d_point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, conn["rb"], { conn["p"] });
			std::vector<symx::Vector> t = this->_get_d_x1(energy, stark, { conn["t0"], conn["t1"], conn["t2"] });
			symx::Scalar d = distance_point_plane(p[0], t[0], t[1], t[2]);
			this->_set_barrier_potential(energy, stark, d);
		}
	);

	//// D -> RB: Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_ep"), this->contacts_rb_deformables.point_triangle.rb_d_edge_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> e = this->_get_rb_x1(energy, stark, conn["rb"], { conn["e0"], conn["e1"] });
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			symx::Scalar d = distance_point_line(p[0], e[0], e[1]);
			this->_set_barrier_potential(energy, stark, d);
		}
	);

	//// D -> RB: Point - Triangle
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "pt_tp"), this->contacts_rb_deformables.point_triangle.rb_d_triangle_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> t = this->_get_rb_x1(energy, stark, conn["rb"], { conn["t0"], conn["t1"], conn["t2"] });
			std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn["p"] });
			symx::Scalar d = distance_point_plane(p[0], t[0], t[1], t[2]);
			this->_set_barrier_potential(energy, stark, d);
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
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
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
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
		}
	);

	//// Edge - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "ee_ee"), this->contacts_rb_deformables.edge_edge.rb_d_edge_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto [ea, ea_rest] = this->_get_rb_edge(energy, stark, conn["rb"], { conn["ea0"], conn["ea1"] });
			auto [eb, eb_rest] = this->_get_d_edge(energy, stark, { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_line_line(ea[0], ea[1], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
		}
	);

	//// D -> RB: Point - Edge
	stark.global_energy.add_energy(this->_get_contact_label("rb_d", "ee_ep"), this->contacts_rb_deformables.edge_edge.rb_d_edge_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			auto [ea, ea_rest] = this->_get_rb_edge(energy, stark, conn["rb"], { conn["ea0"], conn["ea1"] });
			auto [eb, eb_rest, q] = this->_get_d_edge_point(energy, stark, { conn["eb0"], conn["eb1"], conn["q"]});
			symx::Scalar d = distance_point_line(q[0], ea[0], ea[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
		}
	);
}
void stark::EnergyFrictionalContact::_energies_contact_static(core::Stark& stark)
{
	stark.global_energy.add_energy(this->_get_contact_label("s_d", "plane_p"), this->contacts_static.deformable_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector p = energy.make_vector(this->static_planes.point, conn["plane"]);
			symx::Vector n = energy.make_vector(this->static_planes.normal, conn["plane"]);
			symx::Vector a = this->_get_d_x1(energy, stark, { conn["a"] })[0];
			symx::Scalar d = distance_point_plane(a, p, n);
			this->_set_barrier_potential(energy, stark, d);
		}
	);
	stark.global_energy.add_energy(this->_get_contact_label("s_rb", "plane_p"), this->contacts_static.rb_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector p = energy.make_vector(this->static_planes.point, conn["plane"]);
			symx::Vector n = energy.make_vector(this->static_planes.normal, conn["plane"]);
			symx::Vector a = this->_get_rb_x1(energy, stark, conn["rb"], { conn["a"] })[0];
			symx::Scalar d = distance_point_plane(a, p, n);
			this->_set_barrier_potential(energy, stark, d);
		}
	);
}

void stark::EnergyFrictionalContact::_energies_friction_deformables(core::Stark& stark)
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
void stark::EnergyFrictionalContact::_energies_friction_rb(core::Stark& stark)
{
	// Point - Point
	stark.global_energy.add_energy(this->_get_friction_label("rb_rb", "pp"), this->friction_rb.point_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> vp = this->_get_rb_v1(energy, stark, conn["a"], {conn["p"]});
			std::vector<symx::Vector> vq = this->_get_rb_v1(energy, stark, conn["b"], {conn["q"]});
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
			this->_set_friction_edge_edge(energy, stark, vea, veb, conn["idx"], this->friction_deformables.edge_edge.data);
		}
	);
}
void stark::EnergyFrictionalContact::_energies_friction_rb_deformables(core::Stark& stark)
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
void stark::EnergyFrictionalContact::_energies_friction_static(core::Stark& stark)
{
	stark.global_energy.add_energy(this->_get_friction_label("s_d", "plane_p"), this->friction_static.deformable_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector v = this->_get_d_v1(energy, { conn["a"] })[0];
			this->_set_friction_potential(energy, stark, v, conn["idx"], this->friction_static.deformable_point.contact);
		}
	);
	stark.global_energy.add_energy(this->_get_friction_label("s_rb", "plane_p"), this->friction_static.rb_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Vector v = this->_get_rb_v1(energy, stark, conn["rb"], { conn["a"] })[0];
			this->_set_friction_potential(energy, stark, v, conn["idx"], this->friction_static.rb_point.contact);
		}
	);
}


/* ============================================================================= */
/* ===================================  IPC  =================================== */
/* ============================================================================= */
symx::Scalar stark::EnergyFrictionalContact::_barrier_potential(const symx::Scalar& d, const symx::Scalar& dhat, const symx::Scalar& k)
{
	if (this->ipc_barrier_type == IPCBarrierType::Cubic) {
		return k * (dhat - d).powN(3)/3.0;
	}
	else if (this->ipc_barrier_type == IPCBarrierType::Log) {
		return -k * (dhat - d).powN(2) * log(d / dhat);
	}
	else {
		std::cout << "stark error: Unknown IPC barrier type." << std::endl;
		exit(-1);
	}
}
double stark::EnergyFrictionalContact::_barrier_force(const double d, const double dhat, const double k)
{
	if (this->ipc_barrier_type == IPCBarrierType::Cubic) {
		return k * std::pow(dhat - d, 2);
	}
	else if (this->ipc_barrier_type == IPCBarrierType::Log) {
		return (k*(dhat - d)*(2.0*d*std::log(d / dhat) + d - dhat)) / d;
	}
	else {
		std::cout << "stark error: Unknown IPC barrier type." << std::endl;
		exit(-1);
	}
}
symx::Scalar stark::EnergyFrictionalContact::_edge_edge_mollifier(const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest)
{
	symx::Scalar eps_x = 1e-3 * (ea_rest[0] - ea_rest[1]).squared_norm() * (eb_rest[0] - eb_rest[1]).squared_norm();
	symx::Scalar x = (ea[1] - ea[0]).cross3(eb[1] - eb[0]).squared_norm();
	symx::Scalar x_div_eps_x = x / eps_x;
	symx::Scalar f = (-x_div_eps_x + 2.0) * x_div_eps_x;
	symx::Scalar mollifier = symx::branch(x > eps_x, 1.0, f);
	return mollifier;
}
symx::Scalar stark::EnergyFrictionalContact::_friction_potential(const symx::Vector& v, const symx::Scalar& fn, const symx::Scalar& mu, const symx::Matrix& T, const symx::Scalar& epsv, const symx::Scalar& dt)
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
void stark::EnergyFrictionalContact::_set_barrier_potential(symx::Energy& energy, const core::Stark& stark, const symx::Scalar& d)
{
	symx::Scalar k = energy.make_scalar(stark.settings.contact.adaptive_contact_stiffness.value);
	symx::Scalar dhat = energy.make_scalar(stark.settings.contact.dhat);
	symx::Scalar E = this->_barrier_potential(d, dhat, k);
	energy.set(E);
	energy.activate(stark.settings.contact.collisions_enabled);
}
void stark::EnergyFrictionalContact::_set_edge_edge_mollified_barrier_potential(symx::Energy& energy, const core::Stark& stark, const symx::Scalar& d, const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest)
{
	symx::Scalar k = energy.make_scalar(stark.settings.contact.adaptive_contact_stiffness.value);
	symx::Scalar dhat = energy.make_scalar(stark.settings.contact.dhat);
	symx::Scalar E = this->_edge_edge_mollifier(ea, eb, ea_rest, eb_rest) * this->_barrier_potential(d, dhat, k);
	energy.set(E);
	energy.activate(stark.settings.contact.collisions_enabled);
}
void stark::EnergyFrictionalContact::_set_friction_potential(symx::Energy& energy, const core::Stark& stark, const symx::Vector& v, const symx::Index& contact_idx, const FrictionContact& contact)
{
	symx::Matrix T = energy.make_matrix(contact.T, { 2, 3 }, contact_idx);
	symx::Scalar mu = energy.make_scalar(contact.mu, contact_idx);
	symx::Scalar fn = energy.make_scalar(contact.fn, contact_idx);
	symx::Scalar epsv = energy.make_scalar(stark.settings.contact.friction_stick_slide_threshold);
	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
	symx::Scalar E = this->_friction_potential(v, fn, mu, T, epsv, dt);
	energy.set(E);
	energy.activate(stark.settings.contact.collisions_enabled && stark.settings.contact.friction_enabled);
}
void stark::EnergyFrictionalContact::_set_friction_point_edge(symx::Energy& energy, const core::Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& ve, const symx::Index& contact_idx, FrictionPointEdge& data)
{
	symx::Vector bary = energy.make_vector(data.bary, contact_idx);

	symx::Vector va = vp;
	symx::Vector vb = bary[0] * ve[0] + bary[1] * ve[1];
	symx::Vector v = vb - va;
	this->_set_friction_potential(energy, stark, v, contact_idx, data.contact);
}
void stark::EnergyFrictionalContact::_set_friction_point_triangle(symx::Energy& energy, const core::Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& vt, const symx::Index& contact_idx, FrictionPointTriangle& data)
{
	symx::Vector bary = energy.make_vector(data.bary, contact_idx);

	symx::Vector va = vp;
	symx::Vector vb = bary[0] * vt[0] + bary[1] * vt[1] + bary[2] * vt[2];
	symx::Vector v = vb - va;
	this->_set_friction_potential(energy, stark, v, contact_idx, data.contact);
}
void stark::EnergyFrictionalContact::_set_friction_edge_edge(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Vector>& vea, const std::vector<symx::Vector>& veb, const symx::Index& contact_idx, FrictionEdgeEdge& data)
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
std::vector<symx::Vector> stark::EnergyFrictionalContact::_get_rb_v1(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
	std::vector<symx::Vector> x_loc = energy.make_vectors(this->rigidbody_local_vertices.data, conn);
	return this->rb->get_v1(energy, rb_idx, x_loc, dt);
}
std::vector<symx::Vector> stark::EnergyFrictionalContact::_get_rb_x1(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
	std::vector<symx::Vector> x_loc = energy.make_vectors(this->rigidbody_local_vertices.data, conn);
	return this->rb->get_x1(energy, rb_idx, x_loc, dt);
}
std::vector<symx::Vector> stark::EnergyFrictionalContact::_get_rb_X(symx::Energy& energy, const std::vector<symx::Index>& conn)
{
	return energy.make_vectors(this->rigidbody_local_vertices.data, conn);
}
std::array<std::vector<symx::Vector>, 2> stark::EnergyFrictionalContact::_get_rb_edge(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"] }
	std::vector<symx::Vector> ea = this->_get_rb_x1(energy, stark, rb_idx, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_rb_X(energy, { conn[0], conn[1] });
	return { ea, ea_rest };
}
std::array<std::vector<symx::Vector>, 3> stark::EnergyFrictionalContact::_get_rb_edge_point(symx::Energy& energy, const core::Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"], conn["p"] }
	std::vector<symx::Vector> ea = this->_get_rb_x1(energy, stark, rb_idx, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_rb_X(energy, { conn[0], conn[1] });
	std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, rb_idx, { conn[2] });
	return { ea, ea_rest, p };
}
std::vector<symx::Vector> stark::EnergyFrictionalContact::_get_d_v1(symx::Energy& energy, const std::vector<symx::Index>& conn)
{
	return energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, conn);
}
std::vector<symx::Vector> stark::EnergyFrictionalContact::_get_d_x1(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Index>& conn)
{
	std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, conn);
	std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, conn);
	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
	return time_integration(x0, v1, dt);
}
std::vector<symx::Vector> stark::EnergyFrictionalContact::_get_d_X(symx::Energy& energy, const std::vector<symx::Index>& conn)
{
	return energy.make_vectors(this->dyn->X.data, conn);
}
std::array<std::vector<symx::Vector>, 2> stark::EnergyFrictionalContact::_get_d_edge(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"] }
	std::vector<symx::Vector> ea = this->_get_d_x1(energy, stark, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_d_X(energy, { conn[0], conn[1] });
	return { ea, ea_rest };
}
std::array<std::vector<symx::Vector>, 3> stark::EnergyFrictionalContact::_get_d_edge_point(symx::Energy& energy, const core::Stark& stark, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"], conn["p"] }
	std::vector<symx::Vector> ea = this->_get_d_x1(energy, stark, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_d_X(energy, { conn[0], conn[1] });
	std::vector<symx::Vector> p = this->_get_d_x1(energy, stark, { conn[2] });
	return {ea, ea_rest, p};
}


/* ============================================================================== */
/* ===================================  MISC  =================================== */
/* ============================================================================== */
std::string stark::EnergyFrictionalContact::_get_contact_label(const std::string physical_system, const std::string pair) const
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
std::string stark::EnergyFrictionalContact::_get_friction_label(const std::string physical_system, const std::string pair) const
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

