#include "Cloth.h"

#include "../utils/mesh_utils.h"
#include "time_integration.h"
#include "distances.h"
#include "friction_geometry.h"

#include <symx>
#include <vtkio>


void stark::models::Cloth::init(Stark& sim)
{
	this->_init_simulation_structures(sim.settings.execution.n_threads);

	// DoFs
	this->dof = sim.global_energy.add_dof_array(this->model.v1, "cloth_v1");

	// Callbacks
	sim.callbacks.before_time_step.push_back([&]() { this->_before_time_step(sim); });
	sim.callbacks.after_time_step.push_back([&]() { this->_after_time_step(sim); });
	sim.callbacks.write_frame.push_back([&]() { this->_write_frame(sim); });
	sim.callbacks.before_energy_evaluation.push_back([&]() { this->_update_contacts(sim); });
	sim.callbacks.is_state_valid.push_back([&]() { return this->_is_valid_configuration(sim); });

	// Energy declarations
	this->_energies_mechanical(sim);
	this->_energies_contact(sim);
	this->_energies_friction(sim);
}
void stark::models::Cloth::set_vertex_target_position_as_initial(const int cloth_id, const int vertex_id)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->set_vertex_target_position(cloth_id, vertex_id, this->model.mesh.vertices[this->model.mesh.get_global_vertex_idx(cloth_id, vertex_id)]);
}
void stark::models::Cloth::set_vertex_target_position(const int cloth_id, const int vertex_id, const Eigen::Vector3d& position)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->prescribed_nodes_map[this->model.mesh.get_global_vertex_idx(cloth_id, vertex_id)] = position;
	this->changed_prescribed_vertices = true;
}
void stark::models::Cloth::set_attached_vertices(const int cloth_0_id, const int vertex_0_idx, const int cloth_1_id, const int vertex_1_idx)
{
	this->_exit_if_cloth_not_declared(cloth_0_id);
	this->_exit_if_cloth_not_declared(cloth_1_id);
	const int glob_idx_a = this->model.mesh.get_global_vertex_idx(cloth_0_id, vertex_0_idx);
	const int glob_idx_b = this->model.mesh.get_global_vertex_idx(cloth_1_id, vertex_1_idx);
	this->attached_nodes_set.insert({ std::min(glob_idx_a, glob_idx_b), std::max(glob_idx_a, glob_idx_b) });
	this->changed_attachments = true;
}
void stark::models::Cloth::freeze(const int cloth_id)
{
	for (int i = 0; i < this->model.mesh.get_n_vertices(cloth_id); i++) {
		const int idx = this->model.mesh.get_global_vertex_idx(cloth_id, i);
		this->set_vertex_target_position(cloth_id, i, this->model.x1[idx]);
	}
}
void stark::models::Cloth::clear_vertex_target_position()
{
	this->prescribed_nodes_map.clear();
	this->changed_prescribed_vertices = true;
}
void stark::models::Cloth::clear_attached_vertices()
{
	this->attached_nodes_set.clear();
	this->changed_attachments = true;
}
void stark::models::Cloth::set_acceleration(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& acceleration)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->model.set_acceleration(cloth_id, vertex_idx, acceleration);
}
void stark::models::Cloth::add_acceleration(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& acceleration)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->model.add_acceleration(cloth_id, vertex_idx, acceleration);
}
void stark::models::Cloth::set_velocity(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& velocity)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->model.set_velocity(cloth_id, vertex_idx, velocity);
}
void stark::models::Cloth::add_velocity(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& velocity)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->model.add_velocity(cloth_id, vertex_idx, velocity);
}
void stark::models::Cloth::set_position(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& position)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->model.set_position(cloth_id, vertex_idx, position);
}
void stark::models::Cloth::add_displacement(const int cloth_id, const int vertex_idx, const Eigen::Vector3d& displacement)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->model.add_displacement(cloth_id, vertex_idx, displacement);
}
void stark::models::Cloth::clear_acceleration()
{
	std::fill(this->model.a.begin(), this->model.a.end(), Eigen::Vector3d::Zero());
}
void stark::models::Cloth::enable_writing_vtk(const bool write)
{
	this->write_VTK = write;
}
int stark::models::Cloth::add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const MaterialPreset material)
{
	const int cloth_id = this->get_n_cloths();
	this->model.add_mesh(vertices, triangles);

	// Add per-mesh parameters
	//// Inertia
	this->density.push_back(-1.0);

	//// Strain
	this->young_modulus.push_back(-1.0);
	this->poisson_ratio.push_back(-1.0);

	//// Strain limiting
	this->strain_limiting_start.push_back(-1.0);
	this->strain_limiting_stiffness.push_back(-1.0);

	//// Bending
	this->bending_stiffness.push_back(-1.0);

	// Set preset material
	this->set_material_preset(cloth_id, material);
	this->changed_discretization = true;

	return cloth_id;
}
void stark::models::Cloth::set_damping(const double inertial_damping, const double strain_damping, const double bending_damping)
{
	this->inertial_damping = inertial_damping;
	this->strain_damping = strain_damping;
	this->bending_damping = bending_damping;
}
void stark::models::Cloth::set_material_preset(const int cloth_id, const MaterialPreset material)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	switch (material)
	{
	case MaterialPreset::Cotton:
		this->set_density(cloth_id, 0.2);
		this->set_strain_parameters(cloth_id, 30.0, 0.3, 1.1);
		this->set_bending_stiffness(cloth_id, 1e-5);
		break;
	case MaterialPreset::Towel:
		this->set_density(cloth_id, 0.5);
		this->set_strain_parameters(cloth_id, 100.0, 0.3, 0.1, 1.0);
		this->set_bending_stiffness(cloth_id, 5e-6);
		this->set_damping(2.0, 0.2, 0.2);
		break;
	default:
		std::cout << "stark::models::Cloth::set_material_preset() error: cloth material preset not defined." << std::endl;
		exit(-1);
		break;
	}
}
void stark::models::Cloth::set_density(const int cloth_id, const double density)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->density[cloth_id] = density;
	this->changed_discretization = true;
}
void stark::models::Cloth::set_strain_parameters(const int cloth_id, const double young_modulus, const double poisson_ratio, const double strain_limit, const double strain_limit_stiffness)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->young_modulus[cloth_id] = young_modulus;
	this->poisson_ratio[cloth_id] = poisson_ratio;
	this->strain_limiting_start[cloth_id] = strain_limit;
	this->strain_limiting_stiffness[cloth_id] = strain_limit_stiffness;
}
void stark::models::Cloth::set_bending_stiffness(const int cloth_id, const double bending_stiffness)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	this->bending_stiffness[cloth_id] = bending_stiffness;
}
void stark::models::Cloth::set_friction(const int cloth_0, const int cloth_1, const double coulombs_mu)
{
	this->_exit_if_cloth_not_declared(cloth_0);
	this->_exit_if_cloth_not_declared(cloth_1);
	this->mu[{cloth_0, cloth_1}] = coulombs_mu;
	this->mu[{cloth_1, cloth_0}] = coulombs_mu;
}
double stark::models::Cloth::get_friction(const int cloth_0, const int cloth_1)
{
	auto it = this->mu.find({ cloth_0, cloth_1 });
	if (it == this->mu.end()) {
		return 0.0;
	}
	else {
		return it->second;
	}
}
bool stark::models::Cloth::is_cloth_declared(const int cloth_id) const
{
	return cloth_id < this->get_n_cloths();
}
int stark::models::Cloth::get_n_cloths() const
{
	return this->model.mesh.get_n_meshes();
}
bool stark::models::Cloth::is_empty() const
{
	return this->get_n_cloths() == 0;
}
Eigen::Vector3d stark::models::Cloth::get_vertex(const int cloth_id, const int vertex_idx) const
{
	return this->model.x1[this->model.mesh.get_global_vertex_idx(cloth_id, vertex_idx)];
}
Eigen::Vector3d stark::models::Cloth::get_velocity(const int cloth_id, const int vertex_idx) const
{
	return this->model.v1[this->model.mesh.get_global_vertex_idx(cloth_id, vertex_idx)];
}
const stark::utils::TriangleMultiMesh& stark::models::Cloth::get_mesh() const
{
	return this->model.mesh;
}

void stark::models::Cloth::_init_simulation_structures(const int n_threads)
{
	const auto& mesh_rest = this->model.mesh;

	// Inertia
	if (this->changed_discretization) {
		
		// Connectivity
		this->conn_nodes.resize(mesh_rest.get_n_vertices());
		for (int i = 0; i < mesh_rest.get_n_vertices(); i++) {
			this->conn_nodes[i] = {i};
		}

		// Lumped mass
		this->lumped_mass.resize(mesh_rest.get_n_vertices());
		std::fill(this->lumped_mass.begin(), this->lumped_mass.end(), 0.0);
		for (int mesh_i = 0; mesh_i < this->get_n_cloths(); mesh_i++) {
			const double density = this->density[mesh_i];
			mesh_rest.for_each_element_parallel_const(mesh_i,
				[&](const int tri_glob_idx, const std::array<int, 3>& triangle_glob, std::array<const Eigen::Vector3d*, 3>& vertices)
				{
					const double lumped_mass = density*utils::triangle_area(*vertices[0], *vertices[1], *vertices[2]) / 3.0;
					this->lumped_mass[triangle_glob[0]] += lumped_mass;
					this->lumped_mass[triangle_glob[1]] += lumped_mass;
					this->lumped_mass[triangle_glob[2]] += lumped_mass;
				}, n_threads
			);
		}
	}

	// Strain and Strain Limit
	if (this->changed_discretization) {
		this->conn_mesh_numbered_triangles.resize(mesh_rest.get_n_elements());
		this->DXinv.resize(mesh_rest.get_n_elements());
		this->triangle_area_rest.resize(mesh_rest.get_n_elements());
		
		for (int mesh_i = 0; mesh_i < this->get_n_cloths(); mesh_i++) {
			mesh_rest.for_each_element_parallel_const(mesh_i,
				[&](const int tri_glob_idx, const std::array<int, 3>& triangle_glob, std::array<const Eigen::Vector3d*, 3>& vertices)
				{
					// Connectivity
					this->conn_mesh_numbered_triangles[tri_glob_idx] = { tri_glob_idx, mesh_i, triangle_glob[0], triangle_glob[1], triangle_glob[2] };

					const Eigen::Vector3d& p0 = *vertices[0];
					const Eigen::Vector3d& p1 = *vertices[1];
					const Eigen::Vector3d& p2 = *vertices[2];

					// Area
					this->triangle_area_rest[tri_glob_idx] = utils::triangle_area(p0, p1, p2);

					// Projection matrix
					Eigen::Vector3d v01 = p0 - p2;
					Eigen::Vector3d v02 = p1 - p2;
					Eigen::Vector3d px = v01.normalized();
					Eigen::Vector3d normal = v01.cross(v02).normalized();
					Eigen::Vector3d py = normal.cross(px);
					Eigen::Matrix<double, 2, 3> P;
					P.row(0) = px;
					P.row(1) = py;

					const Eigen::Vector2d& p0_ = P * p0;
					const Eigen::Vector2d& p1_ = P * p1;
					const Eigen::Vector2d& p2_ = P * p2;

					Eigen::Matrix2d DX;
					DX.col(0) = p1_ - p0_;
					DX.col(1) = p2_ - p0_;
					Eigen::Matrix2d DXinv_m = DX.inverse();
					this->DXinv[tri_glob_idx] = {
						DXinv_m(0, 0), DXinv_m(0, 1),
						DXinv_m(1, 0), DXinv_m(1, 1)
					};
				}, n_threads
			);
		}
	}

	// Edge strain limiting
	if (this->changed_discretization) {
		this->edges = utils::MultiMeshEdges(this->model.mesh);  // Also used in collision
		this->conn_mesh_numbered_edges.resize(this->edges.get_n_edges());
		this->edge_rest_length.resize(this->edges.get_n_edges());
		for (int edge_i = 0; edge_i < this->edges.get_n_edges(); edge_i++) {
			const std::array<int, 2> edge = this->edges.connectivity[edge_i];
			const int mesh_idx = this->model.mesh.get_mesh_containing_vertex(edge[0]);
			this->conn_mesh_numbered_edges[edge_i] = {edge_i, mesh_idx, edge[0], edge[1]};
			this->edge_rest_length[edge_i] = (this->model.mesh.vertices[edge[1]] - this->model.mesh.vertices[edge[0]]).norm();
		}
	}

	// Bending
	if (this->changed_discretization) {
		// Internal edges connectivity and Bergou06 matrix Q
		auto cotTheta = [](const Eigen::Vector3d& v, const Eigen::Vector3d& w) {
			const double cosTheta = v.dot(w);
			const double sinTheta = (v.cross(w)).norm();
			return (cosTheta / sinTheta);
		};
		const double cutoff_angle_rad = utils::deg2rad(this->cutoff_bending_angle_deg);

		std::vector<std::array<int, 4>> internal_angles;
		utils::find_internal_angles(internal_angles, mesh_rest.connectivity, mesh_rest.get_n_vertices());
		const int n = (int)internal_angles.size();
		this->conn_numbered_mesh_internal_edges.resize(n);
		this->bergou_Q_matrix.resize(n);

		#pragma omp parallel for schedule(static) num_threads(n_threads)
		for (int internal_angle_i = 0; internal_angle_i < n; internal_angle_i++) {
			const std::array<int, 4>& indices = internal_angles[internal_angle_i];

			const Eigen::Vector3d e0 = mesh_rest.vertices[indices[1]] - mesh_rest.vertices[indices[0]];
			const Eigen::Vector3d e1 = mesh_rest.vertices[indices[2]] - mesh_rest.vertices[indices[0]];
			const Eigen::Vector3d e2 = mesh_rest.vertices[indices[3]] - mesh_rest.vertices[indices[0]];
			const Eigen::Vector3d e3 = mesh_rest.vertices[indices[2]] - mesh_rest.vertices[indices[1]];
			const Eigen::Vector3d e4 = mesh_rest.vertices[indices[3]] - mesh_rest.vertices[indices[1]];

			const double c01 = cotTheta(e0, e1);
			const double c02 = cotTheta(e0, e2);
			const double c03 = cotTheta(-e0, e3);
			const double c04 = cotTheta(-e0, e4);

			const Eigen::Vector3d n0 = e0.cross(e1);
			const Eigen::Vector3d n1 = -e0.cross(e2);

			const double angle_rad = std::acos(n0.normalized().dot(n1.normalized()));
			if (angle_rad > cutoff_angle_rad) {
				continue;
			}

			const double A0 = 0.5 * n0.norm();
			const double A1 = 0.5 * n1.norm();

			const double coef = -3.0 / (A0 + A1) * 0.5;
			const Eigen::Vector4d K = Eigen::Vector4d(c03 + c04, c01 + c02, -c01 - c03, -c02 - c04);
			const Eigen::Matrix4d Q = -coef * K * K.transpose();
			this->bergou_Q_matrix[internal_angle_i] = {
				Q(0, 0), Q(0, 1), Q(0, 2), Q(0, 3),
				Q(1, 0), Q(1, 1), Q(1, 2), Q(1, 3),
				Q(2, 0), Q(2, 1), Q(2, 2), Q(2, 3),
				Q(3, 0), Q(3, 1), Q(3, 2), Q(3, 3)
			};
			const int mesh_i = mesh_rest.get_mesh_containing_vertex(indices[0]);
			this->conn_numbered_mesh_internal_edges[internal_angle_i] = { internal_angle_i, mesh_i, indices[0], indices[1], indices[2], indices[3] };
		}
	}

	// Prescribed nodes
	if (this->changed_prescribed_vertices) {
		this->conn_enumerated_prescribed_positions.clear();
		this->prescribed_positions.clear();
		for (const auto& pair : this->prescribed_nodes_map) {
			const int vertex_glob_idx = pair.first;
			const Eigen::Vector3d target_position = pair.second;
			this->conn_enumerated_prescribed_positions.push_back({ (int)this->prescribed_positions.size(), vertex_glob_idx });
			this->prescribed_positions.push_back(target_position);
		}
	}

	// Attachments
	if (this->changed_attachments) {
		this->conn_attached_nodes.clear();
		for (const std::array<int, 2>&pair : this->attached_nodes_set) {
			this->conn_attached_nodes.push_back(pair);
		}
	}

	// Reset
	this->changed_attachments = false;
	this->changed_discretization = false;
	this->changed_prescribed_vertices = false;
}
void stark::models::Cloth::_exit_if_cloth_not_declared(const int cloth_id)
{
	if (!this->is_cloth_declared(cloth_id)) {
		std::cout << "Stark error: There is no cloth with id " + std::to_string(cloth_id) + " declared." << std::endl;
		exit(-1);
	}
}
void stark::models::Cloth::_update_collision_x1(Stark& sim)
{
	const double dt = sim.settings.simulation.adaptive_time_step.value;
	this->collision_x1.resize(this->model.x0.size());
	for (int i = 0; i < this->model.mesh.get_n_vertices(); i++) {
		this->collision_x1[i] = this->model.x0[i] + dt * this->model.v1[i];
	}
}
const tmcd::ProximityResults& stark::models::Cloth::_run_proximity_detection(const std::vector<Eigen::Vector3d>& x, Stark& sim)
{
	if (!this->self_collisions_enabled) {
		return this->pd.get_narrow_phase_results();
	}

	this->pd.clear();
	this->pd.set_n_threads(sim.settings.execution.n_threads);
	this->pd.set_edge_edge_parallel_cutoff(sim.settings.contact.edge_edge_cross_norm_sq_cutoff);
	this->pd.add_mesh(&x[0][0], (int)x.size(), &this->model.mesh.connectivity[0][0], this->model.mesh.get_n_elements(), &this->edges.connectivity[0][0], (int)this->edges.get_n_edges());
	this->pd.activate_point_triangle(sim.settings.contact.triangle_point_enabled);
	this->pd.activate_edge_edge(sim.settings.contact.edge_edge_enabled);
	const tmcd::ProximityResults& proximity = this->pd.run(sim.settings.contact.dhat);
	return proximity;
}

void stark::models::Cloth::_before_time_step(Stark& sim)
{
	if (this->is_empty()) { return; }

	// Reset simulation structures if anything changed
	this->_init_simulation_structures(sim.settings.execution.n_threads);

	// Active friction points at x0 for this time step
	this->_update_friction_contacts(sim);

	// Set next time velocities estimation to zero to avoid invalid state outside of the minimzer
	std::fill(this->model.v1.begin(), this->model.v1.end(), Eigen::Vector3d::Zero());
}
void stark::models::Cloth::_after_time_step(Stark& sim)
{
	if (this->is_empty()) { return; }

	// Set final positions with solved velocities
	const double dt = sim.settings.simulation.adaptive_time_step.value;
	for (int i = 0; i < this->model.mesh.get_n_vertices(); i++) {
		this->model.x1[i] = this->model.x0[i] + dt * this->model.v1[i];
	}

	// x0 <- x1
	this->model.x0 = this->model.x1;
	this->model.v0 = this->model.v1;
}
void stark::models::Cloth::_write_frame(Stark& sim)
{
	if (this->is_empty()) { return; }

	if (this->write_VTK) {
		utils::write_VTK(sim.get_vtk_path("cloth"), this->model.x1, this->model.mesh.connectivity, sim.settings.output.calculate_smooth_normals);
	}
}
bool stark::models::Cloth::_is_valid_configuration(Stark& sim)
{
	if (this->is_empty()) { return true; }
	if (!this->self_collisions_enabled) { return true; }
	if (!sim.settings.contact.collisions_enabled) { return true; }
	if (!sim.settings.contact.enable_intersection_test) { return true; }

	this->_update_collision_x1(sim);
	this->id.clear();
	this->id.set_n_threads(sim.settings.execution.n_threads);
	this->id.add_mesh(&this->collision_x1[0][0], (int)this->collision_x1.size(), &this->model.mesh.connectivity[0][0], this->model.mesh.get_n_elements(), &this->edges.connectivity[0][0], (int)this->edges.get_n_edges());
	const tmcd::IntersectionResults& intersections = this->id.run();
	return intersections.edge_triangle.size() == 0;
}
void stark::models::Cloth::_update_contacts(Stark& sim)
{
	if (this->is_empty()) { return; }
	if (!sim.settings.contact.collisions_enabled) { return; }

	// Proximity detection
	this->_update_collision_x1(sim);
	const tmcd::ProximityResults& proximity = this->_run_proximity_detection(this->collision_x1, sim);

	// Fill connectivities
	this->contacts.clear();

	//// Point - Triangle
	for (const auto& pair : proximity.point_triangle.point_point) {
		const tmcd::Point& p = pair.first;
		const tmcd::TrianglePoint& tp = pair.second;
		const tmcd::Point& q = tp.point;
		this->contacts.point_triangle.point_point.push_back({ p.idx, q.idx });
	}
	for (const auto& pair : proximity.point_triangle.point_edge) {
		const tmcd::Point& point = pair.first;
		const tmcd::TriangleEdge& te = pair.second;
		const tmcd::TriangleEdge::Edge& edge = te.edge;
		this->contacts.point_triangle.point_edge.push_back({ point.idx, edge.vertices[0], edge.vertices[1] });
	}
	for (const auto& pair : proximity.point_triangle.point_triangle) {
		const tmcd::Point& point = pair.first;
		const tmcd::Triangle& triangle = pair.second;
		this->contacts.point_triangle.point_triangle.push_back({ point.idx, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2] });
	}

	//// Edge - Edge
	for (const auto& pair : proximity.edge_edge.point_point) {
		const tmcd::EdgePoint& ep_a = pair.first;
		const tmcd::EdgePoint& ep_b = pair.second;
		this->contacts.edge_edge.point_point.push_back({ ep_a.edge.vertices[0], ep_a.edge.vertices[1], ep_a.point.idx, ep_b.edge.vertices[0], ep_b.edge.vertices[1], ep_b.point.idx });
	}
	for (const auto& pair : proximity.edge_edge.point_edge) {
		const tmcd::EdgePoint& ep_a = pair.first;
		const tmcd::Edge& e_b = pair.second;
		this->contacts.edge_edge.point_edge.push_back({ ep_a.edge.vertices[0], ep_a.edge.vertices[1], ep_a.point.idx, e_b.vertices[0], e_b.vertices[1] });
	}
	for (const auto& pair : proximity.edge_edge.edge_edge) {
		this->contacts.edge_edge.edge_edge.push_back({ pair.first.vertices[0], pair.first.vertices[1], pair.second.vertices[0], pair.second.vertices[1] });
	}
}
void stark::models::Cloth::_update_friction_contacts(Stark& sim)
{
	if (!sim.settings.contact.collisions_enabled) { return; }
	if (!sim.settings.contact.friction_enabled) { return; }

	// Proximity detection
	const std::vector<Eigen::Vector3d>& x = this->model.x0;
	const tmcd::ProximityResults& proximity = this->_run_proximity_detection(x, sim);

	// Friction force
	const double k = sim.settings.contact.adaptive_contact_stiffness.value;
	const double dhat = sim.settings.contact.dhat;
	auto force = [&](double d) { return k * 3.0*std::pow(dhat - d, 2); };

	// Fill friction data
	this->friction.clear();
	
	// Point - Triangle
	//// Point - Point
	for (const auto& pair : proximity.point_triangle.point_point) {
		const tmcd::Point& p = pair.first;
		const tmcd::TrianglePoint& tp = pair.second;
		const tmcd::Point& q = tp.point;

		const int cloth_a_idx = this->model.mesh.get_mesh_containing_vertex(p.idx);
		const int cloth_b_idx = this->model.mesh.get_mesh_containing_vertex(q.idx);
		const double mu = this->get_friction(cloth_a_idx, cloth_b_idx);
		if (mu > 0.0) {
			this->friction.point_point.conn.push_back({ (int)this->friction.point_point.conn.size(), p.idx, q.idx});
			this->friction.point_point.contact.T.push_back(projection_matrix_point_point(x[p.idx], x[q.idx]));
			this->friction.point_point.contact.mu.push_back(mu);
			this->friction.point_point.contact.fn.push_back(force(pair.distance));
		}
	}
	//// Point - Edge
	for (const auto& pair : proximity.point_triangle.point_edge) {
		const tmcd::Point& p = pair.first;
		const tmcd::TriangleEdge& te = pair.second;
		const tmcd::TriangleEdge::Edge& edge = te.edge;

		const int cloth_a_idx = this->model.mesh.get_mesh_containing_vertex(p.idx);
		const int cloth_b_idx = this->model.mesh.get_mesh_containing_vertex(edge.vertices[0]);
		const double mu = this->get_friction(cloth_a_idx, cloth_b_idx);
		if (mu > 0.0) {
			this->friction.point_edge.conn.push_back({ (int)this->friction.point_edge.conn.size(), p.idx, edge.vertices[0], edge.vertices[1] });
			this->friction.point_edge.bary.push_back(barycentric_point_edge(x[p.idx], x[edge.vertices[0]], x[edge.vertices[1]]));
			this->friction.point_edge.contact.T.push_back(projection_matrix_point_edge(x[p.idx], x[edge.vertices[0]], x[edge.vertices[1]]));
			this->friction.point_edge.contact.mu.push_back(mu);
			this->friction.point_edge.contact.fn.push_back(force(pair.distance));
		}
	}
	//// Point - Triangle
	for (const auto& pair : proximity.point_triangle.point_triangle) {
		const tmcd::Point& p = pair.first;
		const tmcd::Triangle& t = pair.second;

		const int cloth_a_idx = this->model.mesh.get_mesh_containing_vertex(p.idx);
		const int cloth_b_idx = this->model.mesh.get_mesh_containing_vertex(t.vertices[0]);
		const double mu = this->get_friction(cloth_a_idx, cloth_b_idx);
		if (mu > 0.0) {
			this->friction.point_triangle.conn.push_back({ (int)this->friction.point_triangle.conn.size(), p.idx, t.vertices[0], t.vertices[1], t.vertices[2] });
			this->friction.point_triangle.bary.push_back(barycentric_point_triangle(x[p.idx], x[t.vertices[0]], x[t.vertices[1]], x[t.vertices[2]]));
			this->friction.point_triangle.contact.T.push_back(projection_matrix_triangle(x[t.vertices[0]], x[t.vertices[1]], x[t.vertices[2]]));
			this->friction.point_triangle.contact.mu.push_back(mu);
			this->friction.point_triangle.contact.fn.push_back(force(pair.distance));
		}
	}


	// Edge - Edge
	auto are_not_almost_parallel = [&](const tmcd::Edge& edge_a, const tmcd::Edge& edge_b) 
	{
		const Eigen::Vector3d& ea0 = x[edge_a.vertices[0]];
		const Eigen::Vector3d& ea1 = x[edge_a.vertices[1]];
		const Eigen::Vector3d& eb0 = x[edge_b.vertices[0]];
		const Eigen::Vector3d& eb1 = x[edge_b.vertices[1]];
		return (ea1 - ea0).cross(eb1 - eb0).squaredNorm() > 1e-16;
	};
	auto mollifier = [&](const tmcd::Edge& edge_a, const tmcd::Edge& edge_b) 
	{
		const std::vector<Eigen::Vector3d>& X = this->model.X;
		return edge_edge_mollifier(x[edge_a.vertices[0]], x[edge_a.vertices[1]], x[edge_b.vertices[0]], x[edge_b.vertices[1]],
								   X[edge_a.vertices[0]], X[edge_a.vertices[1]], X[edge_b.vertices[0]], X[edge_b.vertices[1]]);
	};

	//// Point - Point
	for (const auto& pair : proximity.edge_edge.point_point) {
		const tmcd::EdgePoint& ep_a = pair.first;
		const tmcd::EdgePoint& ep_b = pair.second;
		const tmcd::Point& p = ep_a.point;
		const tmcd::Point& q = ep_b.point;

		const int cloth_a_idx = this->model.mesh.get_mesh_containing_vertex(p.idx);
		const int cloth_b_idx = this->model.mesh.get_mesh_containing_vertex(q.idx);
		const double mu = this->get_friction(cloth_a_idx, cloth_b_idx);
		if (mu > 0.0 && are_not_almost_parallel(ep_a.edge, ep_b.edge)) {
			this->friction.point_point.conn.push_back({ (int)this->friction.point_point.conn.size(), p.idx, q.idx });
			this->friction.point_point.contact.T.push_back(projection_matrix_point_point(x[p.idx], x[q.idx]));
			this->friction.point_point.contact.mu.push_back(mu);
			this->friction.point_point.contact.fn.push_back(mollifier(ep_a.edge, ep_b.edge)*force(pair.distance));
		}
	}
	//// Point - Edge
	for (const auto& pair : proximity.edge_edge.point_edge) {
		const tmcd::EdgePoint& ep = pair.first;
		const tmcd::Point& p = ep.point;
		const tmcd::Edge& edge = pair.second;

		const int cloth_a_idx = this->model.mesh.get_mesh_containing_vertex(p.idx);
		const int cloth_b_idx = this->model.mesh.get_mesh_containing_vertex(edge.vertices[0]);
		const double mu = this->get_friction(cloth_a_idx, cloth_b_idx);
		if (mu > 0.0 && are_not_almost_parallel(ep.edge, edge)) {
			this->friction.point_edge.conn.push_back({ (int)this->friction.point_edge.conn.size(), p.idx, edge.vertices[0], edge.vertices[1] });
			this->friction.point_edge.bary.push_back(barycentric_point_edge(x[p.idx], x[edge.vertices[0]], x[edge.vertices[1]]));
			this->friction.point_edge.contact.T.push_back(projection_matrix_point_edge(x[p.idx], x[edge.vertices[0]], x[edge.vertices[1]]));
			this->friction.point_edge.contact.mu.push_back(mu);
			this->friction.point_edge.contact.fn.push_back(mollifier(ep.edge, edge) * force(pair.distance));
		}
	}
	//// Edge - Edge
	for (const auto& pair : proximity.edge_edge.edge_edge) {
		const tmcd::Edge& ea = pair.first;
		const tmcd::Edge& eb = pair.second;

		const int cloth_a_idx = this->model.mesh.get_mesh_containing_vertex(ea.vertices[0]);
		const int cloth_b_idx = this->model.mesh.get_mesh_containing_vertex(eb.vertices[0]);
		const double mu = this->get_friction(cloth_a_idx, cloth_b_idx);
		if (mu > 0.0 && are_not_almost_parallel(ea, eb)) {
			this->friction.edge_edge.conn.push_back({ (int)this->friction.edge_edge.conn.size(), ea.vertices[0], ea.vertices[1], eb.vertices[0], eb.vertices[1] });
			this->friction.edge_edge.bary.push_back(barycentric_edge_edge(x[ea.vertices[0]], x[ea.vertices[1]], x[eb.vertices[0]], x[eb.vertices[1]]));
			this->friction.edge_edge.contact.T.push_back(projection_matrix_edge_edge(x[ea.vertices[0]], x[ea.vertices[1]], x[eb.vertices[0]], x[eb.vertices[1]]));
			this->friction.edge_edge.contact.mu.push_back(mu);
			this->friction.edge_edge.contact.fn.push_back(mollifier(ea, eb) * force(pair.distance));
		}
	}
}

void stark::models::Cloth::_energies_mechanical(Stark& sim)
{
	// Lumped mass inertia
	sim.global_energy.add_energy("cloth_inertia", this->conn_nodes,
		[&](symx::Energy& energy, symx::Element& node)
		{
			//// Create symbols
			symx::Vector v1 = energy.make_dof_vector(dof, this->model.v1, node[0]);
			symx::Vector x0 = energy.make_vector(this->model.x0, node[0]);
			symx::Vector v0 = energy.make_vector(this->model.v0, node[0]);
			symx::Vector a = energy.make_vector(this->model.a, node[0]);
			symx::Scalar mass = energy.make_scalar(this->lumped_mass, node[0]);

			symx::Scalar inertial_damping = energy.make_scalar(this->inertial_damping);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Vector gravity = energy.make_vector(sim.settings.simulation.gravity);

			//// Set energy expression
			symx::Vector x1 = x0 + dt * v1;
			symx::Vector xhat = x0 + dt * v0 + dt * dt * (a + gravity);
			symx::Vector dev = x1 - xhat;
			symx::Vector dev2 = x1 - x0;
			symx::Scalar E = 0.5 * mass * (dev.dot(dev) / (dt.powN(2)) + dev2.dot(dev2) * inertial_damping / dt);
			energy.set(E);
		}
	);

	// Strain
	sim.global_energy.add_energy("cloth_strain", this->conn_mesh_numbered_triangles,
		[&](symx::Energy& energy, symx::Element& mesh_idx_triangle)
		{
			// Unpack connectivity
			symx::Index tri_idx = mesh_idx_triangle[0];
			symx::Index mesh_idx = mesh_idx_triangle[1];
			std::vector<symx::Index> triangle = mesh_idx_triangle.slice(2, 5);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(dof, this->model.v1, triangle);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, triangle);
			symx::Matrix DXinv = energy.make_matrix(this->DXinv, { 2, 2 }, tri_idx);
			symx::Scalar area = energy.make_scalar(this->triangle_area_rest, tri_idx);
			symx::Scalar E = energy.make_scalar(this->young_modulus, mesh_idx);
			symx::Scalar nu = energy.make_scalar(this->poisson_ratio, mesh_idx);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Kinematics
			symx::Matrix Dx = symx::Matrix(symx::gather({
				x1[1] - x1[0],
				x1[2] - x1[0],
				}), { 2, 3 }).transpose();
			symx::Matrix F_32 = Dx * DXinv;  // 3x2
			symx::Matrix C = F_32.transpose() * F_32;

			// Strain energy
			symx::Scalar mu = E / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (E * nu) / ((1.0 + nu) * (1.0 - nu));  // 2D !!
			symx::Scalar def_area = 0.5 * ((x1[0] - x1[2]).cross3(x1[1] - x1[2])).norm();
			symx::Scalar J = def_area / area;
			symx::Scalar Ic = C.trace();
			symx::Scalar logJ = symx::log(J);
			symx::Scalar energy_density = 0.5 * mu * (Ic - 3.0) - mu * logJ + 0.5 * lambda * logJ.powN(2);
			symx::Scalar Energy = area * energy_density;
			energy.set(Energy);
		}
	);

	// Strain damping
	sim.global_energy.add_energy("cloth_strain_damping", this->conn_mesh_numbered_edges,
		[&](symx::Energy& energy, symx::Element& mesh_idx_edge)
		{
			// Unpack connectivity
			symx::Index edge_idx = mesh_idx_edge[0];
			symx::Index mesh_idx = mesh_idx_edge[1];
			std::vector<symx::Index> edge = mesh_idx_edge.slice(2, 4);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(dof, this->model.v1, edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, edge);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Scalar l0 = energy.make_scalar(this->edge_rest_length, edge_idx);
			symx::Scalar damping = energy.make_scalar(this->strain_damping);
			symx::Scalar E = energy.make_scalar(this->young_modulus, mesh_idx);
			symx::Scalar nu = energy.make_scalar(this->poisson_ratio, mesh_idx);

			// Set energy expression
			symx::Scalar mu = E / (2.0 * (1.0 + nu));
			symx::Vector u = (x0[1] - x0[0]).normalized();
			symx::Vector dv = v1[1] - v1[0];
			symx::Scalar Energy = l0*damping*mu*dt*(u.transpose()*dv).powN(2);  // f = dE/dx = -l0*damping*(u.T*dv)u
			energy.set(Energy);
		}
	);


	
	//// Strain limiting
	//sim.global_energy.add_energy("cloth_strain_limiting", this->conn_mesh_numbered_triangles,
	//	[&](symx::Energy& energy, symx::Element& mesh_idx_triangle)
	//	{
	//		// Unpack connectivity
	//		symx::Index tri_idx = mesh_idx_triangle[0];
	//		symx::Index mesh_idx = mesh_idx_triangle[1];
	//		std::vector<symx::Index> triangle = mesh_idx_triangle.slice(2, 5);

	//		// Create symbols
	//		std::vector<symx::Vector> v1 = energy.make_dof_vectors(dof, this->model.v1, triangle);
	//		std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, triangle);
	//		symx::Matrix DXinv = energy.make_matrix(this->DXinv, { 2, 2 }, tri_idx);
	//		symx::Scalar area = energy.make_scalar(this->triangle_area_rest, tri_idx);
	//		symx::Scalar strain_limiting_start = energy.make_scalar(this->strain_limiting_start, mesh_idx);
	//		symx::Scalar strain_limiting_stiffness = energy.make_scalar(this->strain_limiting_stiffness, mesh_idx);
	//		symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

	//		// Time integration
	//		std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

	//		// Kinematics
	//		symx::Matrix Dx = symx::Matrix(symx::gather({
	//			x1[1] - x1[0],
	//			x1[2] - x1[0],
	//			}), { 2, 3 }).transpose();
	//		symx::Matrix F_32 = Dx * DXinv;  // 3x2
	//		symx::Matrix C = F_32.transpose() * F_32;

	//		symx::Vector s = C.singular_values_2x2();
	//		symx::Scalar constraint = symx::sqrt(s[0]) - strain_limiting_start;
	//		symx::Scalar E = area * strain_limiting_stiffness * constraint.powN(3);
	//		energy.set_with_condition(E, constraint > 0.0);
	//		energy.activate(this->is_strain_limiting_active);
	//	}
	//);
	
	// Strain limiting
	sim.global_energy.add_energy("cloth_edge_strain_limiting", this->conn_mesh_numbered_edges, // [edge_idx, mesh_idx, edge]
		[&](symx::Energy& energy, symx::Element& mesh_idx_edge)
		{
			// Unpack connectivity
			symx::Index edge_idx = mesh_idx_edge[0];
			symx::Index mesh_idx = mesh_idx_edge[1];
			std::vector<symx::Index> edge = mesh_idx_edge.slice(2, 4);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(dof, this->model.v1, edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, edge);
			symx::Scalar l0 = energy.make_scalar(this->edge_rest_length, edge_idx);
			symx::Scalar strain_limiting_start = energy.make_scalar(this->strain_limiting_start, mesh_idx);
			symx::Scalar strain_limiting_stiffness = energy.make_scalar(this->strain_limiting_stiffness, mesh_idx);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Constraint
			symx::Scalar l = (x1[0] - x1[1]).norm();
			symx::Scalar s = (l - l0) / l0;
			symx::Scalar C = s - strain_limiting_start;
			symx::Scalar E = l0 * strain_limiting_stiffness * C.powN(3);
			energy.set_with_condition(E, C > 0.0);
			energy.activate(this->is_strain_limiting_active);
		}
	);

	// Bergou06 Bending Energy
	sim.global_energy.add_energy("cloth_bending", this->conn_numbered_mesh_internal_edges,
		[&](symx::Energy& energy, symx::Element& mesh_idx_internal_edge)
		{
			// Unpack connectivity
			symx::Index ie_idx = mesh_idx_internal_edge[0];
			symx::Index mesh_idx = mesh_idx_internal_edge[1];
			std::vector<symx::Index> internal_edge = mesh_idx_internal_edge.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(dof, this->model.v1, internal_edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, internal_edge);
			symx::Matrix Q = energy.make_matrix(this->bergou_Q_matrix, { 4, 4 }, ie_idx);
			symx::Scalar stiffness = energy.make_scalar(this->bending_stiffness, mesh_idx);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			symx::Scalar damping = energy.make_scalar(this->bending_damping);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = dt.get_zero();
			for (int i = 0; i < 3; i++) {
				symx::Vector x = symx::Vector({ x1[0][i], x1[1][i], x1[2][i], x1[3][i] });
				symx::Vector v = symx::Vector({ v1[0][i], v1[1][i], v1[2][i], v1[3][i] });
				E += stiffness*(x.transpose()*Q*x) + 0.5*damping*stiffness*dt*(v.transpose()*Q*v);
			}
			energy.set(E);
		}
	);
	
	// Prescribed nodes
	sim.global_energy.add_energy("cloth_prescribed_positions", this->conn_enumerated_prescribed_positions,
		[&](symx::Energy& energy, symx::Element& node)
		{
			// Unpack connectivity
			symx::Index constraint_idx = node[0];
			symx::Index node_idx = node[1];

			//// Create symbols
			symx::Vector v1 = energy.make_dof_vector(dof, this->model.v1, node_idx);
			symx::Vector x0 = energy.make_vector(this->model.x0, node_idx);
			symx::Vector x1_prescribed = energy.make_vector(this->prescribed_positions, constraint_idx);
			symx::Scalar k = energy.make_scalar(sim.settings.simulation.boundary_conditions_stiffness);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			symx::Vector x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1 - x1_prescribed).squared_norm();
			energy.set(E);
		}
	);

	// Attachments
	sim.global_energy.add_energy("cloth_attachments", this->conn_attached_nodes,
		[&](symx::Energy& energy, symx::Element& node_pair)
		{
			//// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(dof, this->model.v1, node_pair);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, node_pair);
			symx::Scalar k = energy.make_scalar(sim.settings.simulation.boundary_conditions_stiffness);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1[0] - x1[1]).squared_norm();
			energy.set(E);
		}
	);
}
void stark::models::Cloth::_energies_contact(Stark& sim)
{
	// Common symbol creation and manipulation functions ------------------------
	auto get_x1 = [&](const std::vector<symx::Index>& conn, symx::Energy& energy)
	{
		std::vector<symx::Vector> v1 = energy.make_dof_vectors(dof, this->model.v1, conn);
		std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, conn);
		symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
		return time_integration(x0, v1, dt);
	};
	auto get_X = [&](const std::vector<symx::Index>& conn, symx::Energy& energy)
	{
		return energy.make_vectors(this->model.X, conn);
	};
	auto barrier_energy = [&](const symx::Scalar& d, const symx::Scalar& dhat, const symx::Scalar& k)
	{
		return k * (dhat - d).powN(3);
	};
	auto set_barrier_energy = [&](const symx::Scalar& d, symx::Energy& energy)
	{
		symx::Scalar k = energy.make_scalar(sim.settings.contact.adaptive_contact_stiffness.value);
		symx::Scalar dhat = energy.make_scalar(sim.settings.contact.dhat);
		symx::Scalar E = barrier_energy(d, dhat, k);
		energy.set(E);
	};
	auto edge_edge_mollifier = [&](const std::vector<symx::Vector>& EA, const std::vector<symx::Vector>& EB, const std::vector<symx::Vector>& EA_REST, const std::vector<symx::Vector>& EB_REST)
	{
		symx::Scalar eps_x = 1e-3 * (EA_REST[0] - EA_REST[1]).squared_norm() * (EB_REST[0] - EB_REST[1]).squared_norm();
		symx::Scalar x = (EA[1] - EA[0]).cross3(EB[1] - EB[0]).squared_norm();
		symx::Scalar x_div_eps_x = x / eps_x;
		symx::Scalar f = (-x_div_eps_x + 2.0) * x_div_eps_x;
		symx::Scalar mollifier = symx::branch(x > eps_x, 1.0, f);
		return mollifier;
	};
	auto set_edge_dge_mollified_barrier_energy = [&](const std::vector<symx::Vector>& EA, const std::vector<symx::Vector>& EB, const std::vector<symx::Vector>& EA_REST, const std::vector<symx::Vector>& EB_REST, const symx::Scalar& d, symx::Energy& energy)
	{
		symx::Scalar k = energy.make_scalar(sim.settings.contact.adaptive_contact_stiffness.value);
		symx::Scalar dhat = energy.make_scalar(sim.settings.contact.dhat);
		symx::Scalar E = edge_edge_mollifier(EA, EB, EA_REST, EB_REST)*barrier_energy(d, dhat, k);
		energy.set(E);
	};
	// -----------------------------------------------------------------------------

	// Point - Triangle
	//// Point - Point
	sim.global_energy.add_energy("collision_cloth_cloth_pt_point_point", this->contacts.point_triangle.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> P = get_x1({ conn[0] }, energy);
			std::vector<symx::Vector> Q = get_x1({ conn[1] }, energy);
			symx::Scalar d = distance_point_point(P[0], Q[0]);
			set_barrier_energy(d, energy);
		}
	);

	//// Point - Edge
	sim.global_energy.add_energy("collision_cloth_cloth_pt_point_edge", this->contacts.point_triangle.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> P = get_x1({ conn[0] }, energy);
			std::vector<symx::Vector> Q = get_x1({ conn[1], conn[2] }, energy);
			symx::Scalar d = distance_point_line(P[0], Q[0], Q[1]);
			set_barrier_energy(d, energy);
		}
	);

	//// Point - Triangle
	sim.global_energy.add_energy("collision_cloth_cloth_pt_point_triangle", this->contacts.point_triangle.point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> P = get_x1({ conn[0] }, energy);
			std::vector<symx::Vector> Q = get_x1({ conn[1], conn[2], conn[3] }, energy);
			symx::Scalar d = distance_point_plane(P[0], Q[0], Q[1], Q[2]);
			set_barrier_energy(d, energy);
		}
	);

		
	// Edge - Edge
	//// Point - Point
	sim.global_energy.add_energy("collision_cloth_cloth_ee_point_point", this->contacts.edge_edge.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> EA = get_x1({ conn[0], conn[1] }, energy);
			std::vector<symx::Vector> EA_REST = get_X({ conn[0], conn[1] }, energy);
			std::vector<symx::Vector> PA = get_x1({ conn[2] }, energy);
			std::vector<symx::Vector> EB = get_x1({ conn[3], conn[4] }, energy);
			std::vector<symx::Vector> EB_REST = get_X({ conn[3], conn[4] }, energy);
			std::vector<symx::Vector> PB = get_x1({ conn[5] }, energy);
			symx::Scalar d = distance_point_point(PA[0], PB[0]);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);

	//// Point - Edge
	sim.global_energy.add_energy("collision_cloth_cloth_ee_point_edge", this->contacts.edge_edge.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> EA = get_x1({ conn[0], conn[1] }, energy);
			std::vector<symx::Vector> EA_REST = get_X({ conn[0], conn[1] }, energy);
			std::vector<symx::Vector> PA = get_x1({ conn[2] }, energy);
			std::vector<symx::Vector> EB = get_x1({ conn[3], conn[4] }, energy);
			std::vector<symx::Vector> EB_REST = get_X({ conn[3], conn[4] }, energy);
			symx::Scalar d = distance_point_line(PA[0], EB[0], EB[1]);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);

	//// Edge - Edge
	sim.global_energy.add_energy("collision_cloth_cloth_ee_edge_edge", this->contacts.edge_edge.edge_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> EA = get_x1({ conn[0], conn[1] }, energy);
			std::vector<symx::Vector> EA_REST = get_X({ conn[0], conn[1] }, energy);
			std::vector<symx::Vector> EB = get_x1({ conn[2], conn[3] }, energy);
			std::vector<symx::Vector> EB_REST = get_X({ conn[2], conn[3] }, energy);
			symx::Scalar d = distance_line_line(EA[0], EA[1], EB[0], EB[1]);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);
}
void stark::models::Cloth::_energies_friction(Stark& sim)
{
	// Common symbol creation and manipulation functions --------------------------------------------------------------------------------------------------
	auto get_v1 = [&](const std::vector<symx::Index>& conn, symx::Energy& energy)
	{
		return energy.make_dof_vectors(this->dof, this->model.v1, conn);
	};
	auto set_friction_energy = [&](const symx::Vector& v, const symx::Index& contact_idx, const TriangleMeshFriction::Contact& contact, symx::Energy& energy)
	{
		symx::Matrix T  = energy.make_matrix(contact.T, { 2, 3 }, contact_idx);
		symx::Scalar mu = energy.make_scalar(contact.mu, contact_idx);
		symx::Scalar fn = energy.make_scalar(contact.fn, contact_idx);
		symx::Scalar epsv = energy.make_scalar(sim.settings.contact.friction_stick_slide_threshold);
		symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
		symx::Scalar perturbation = energy.make_scalar(sim.settings.contact.friction_displacement_perturbation);

		symx::Vector vt = T*v;
		symx::Vector ut = vt*dt;
		ut[0] += 1.13*perturbation;
		ut[1] -= 1.07*perturbation;
		symx::Scalar u = ut.norm();

		symx::Scalar epsu = dt*epsv;
		if (sim.settings.contact.better_friction_mode) {
			symx::Scalar k = mu*fn/epsu;
			symx::Scalar eps = mu*fn/(2.0*k);

			symx::Scalar E_stick = 0.5*k*u.powN(2);
			symx::Scalar E_slide = mu*fn*(u - eps);
			symx::Scalar E = symx::branch(u < epsu, E_stick, E_slide);
			energy.set(E);
		}
		else {
			symx::Scalar E_stick = mu*fn*(-u*u*u/(3.0*epsu.powN(2)) + u*u/epsu + epsu/3.0);
			symx::Scalar E_slide = mu*fn*u;
			symx::Scalar E = symx::branch(u < epsu, E_stick, E_slide);
			energy.set(E);
		}
	};
	// ----------------------------------------------------------------------------------------------------------------------------------------------------


	// Point - Point
	sim.global_energy.add_energy("friction_cloth_cloth_point_point", this->friction.point_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> VP = get_v1({ conn[1] }, energy);
			std::vector<symx::Vector> VQ = get_v1({ conn[2] }, energy);

			symx::Vector v = VQ[0] - VP[0];
			set_friction_energy(v, conn[0], this->friction.point_point.contact, energy);
		}
	);
	// Point - Edge
	sim.global_energy.add_energy("friction_cloth_cloth_point_edge", this->friction.point_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> VP = get_v1({ conn[1] }, energy);
			std::vector<symx::Vector> VE = get_v1({ conn[2], conn[3] }, energy);
			symx::Vector bary = energy.make_vector(this->friction.point_edge.bary, conn[0]);

			symx::Vector vp = VP[0];
			symx::Vector vq = bary[0]*VE[0] + bary[1]*VE[1];
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn[0], this->friction.point_edge.contact, energy);
		}
	);
	// Point - Triangle
	sim.global_energy.add_energy("friction_cloth_cloth_point_triangle", this->friction.point_triangle.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> VP = get_v1({ conn[1] }, energy);
			std::vector<symx::Vector> VT = get_v1({ conn[2], conn[3], conn[4] }, energy);
			symx::Vector bary = energy.make_vector(this->friction.point_triangle.bary, conn[0]);

			symx::Vector vp = VP[0];
			symx::Vector vq = bary[0]*VT[0] + bary[1]*VT[1] + bary[2]*VT[2];  
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn[0], this->friction.point_triangle.contact, energy);
		}
	);
	// Edge - Edge
	sim.global_energy.add_energy("friction_cloth_cloth_edge_edge", this->friction.edge_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> VEA = get_v1({ conn[1], conn[2] }, energy);
			std::vector<symx::Vector> VEB = get_v1({ conn[3], conn[4] }, energy);
			symx::Vector bary = energy.make_vector(this->friction.edge_edge.bary, conn[0]);

			symx::Vector vp = VEA[0] + bary[0]*(VEA[1] - VEA[0]);
			symx::Vector vq = VEB[0] + bary[1]*(VEB[1] - VEB[0]);
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn[0], this->friction.edge_edge.contact, energy);
		}
	);
}
