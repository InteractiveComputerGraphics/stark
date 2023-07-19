#include "Cloth.h"

#include "../utils/Output.h"
#include "../utils/mesh_utils.h"
#include "time_integration.h"
#include "distances.h"

void stark::models::Cloth::init(Simulation& sim)
{
	Output::output->cout("Cloth::init()\n", 3);

	this->_init_simulation_structures(sim.options.n_threads);

	// DoFs
	sim.global_energy.add_dof_array(this->model.v1);

	// Callbacks
	sim.callbacks.before_time_step.push_back([&](Simulation& sim) { this->_before_time_step(sim); });
	sim.callbacks.after_time_step.push_back([&](Simulation& sim) { this->_after_time_step(sim); });
	sim.callbacks.write_frame.push_back([&](Simulation& sim) { this->_write_frame(sim); });
	sim.callbacks.before_energy_evaluation.push_back([&](Simulation& sim) { this->_update_contacts(sim); });
	sim.callbacks.is_state_valid.push_back([&](Simulation& sim) { return this->_is_valid_configuration(sim); });

	// Energies
	//// Lumped mass inertia
	sim.global_energy.add_energy("cloth_inertia", this->conn_nodes,
		[&](symx::Energy& energy, symx::Element& node)
		{
			//// Create symbols
			symx::Scalar mass = energy.make_scalar(this->lumped_mass, node[0]);
			symx::Vector x0 = energy.make_vector(this->model.x0, node[0]);
			symx::Vector v0 = energy.make_vector(this->model.v0, node[0]);
			symx::Vector v1 = energy.make_vector(this->model.v1, node[0]);
			symx::Vector a = energy.make_vector(this->model.a, node[0]);

			symx::Scalar damping = energy.make_scalar(this->damping);
			symx::Scalar dt = energy.make_scalar(sim.parameters.time_step.value);
			symx::Vector gravity = energy.make_vector(sim.parameters.gravity);

			//// Set energy expression
			symx::Vector x1 = x0 + dt * v1;
			symx::Vector xhat = x0 + dt * v0 + dt * dt * (a + gravity);
			symx::Vector dev = x1 - xhat;
			symx::Vector dev2 = x1 - x0;
			symx::Scalar E = 0.5 * mass * (dev.dot(dev) / (dt.powN(2)) + dev2.dot(dev2) * damping / dt);
			energy.set_expression(E);
		}
	);

	//// Strain
	sim.global_energy.add_energy("cloth_strain", this->conn_mesh_numbered_triangles,
		[&](symx::Energy& energy, symx::Element& mesh_idx_triangle)
		{
			// Unpack connectivity
			symx::Index tri_idx = mesh_idx_triangle[0];
			symx::Index mesh_idx = mesh_idx_triangle[1];
			std::vector<symx::Index> triangle = mesh_idx_triangle.slice(2, 5);

			// Create symbols
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, triangle);
			std::vector<symx::Vector> v1 = energy.make_vectors(this->model.v1, triangle);
			symx::Matrix DXinv = energy.make_matrix(this->DXinv, { 2, 2 }, tri_idx);
			symx::Scalar area = energy.make_scalar(this->triangle_area_rest, tri_idx);
			symx::Scalar E = energy.make_scalar(this->young_modulus, mesh_idx);
			symx::Scalar nu = energy.make_scalar(this->poisson_ratio, mesh_idx);
			symx::Scalar dt = energy.make_scalar(sim.parameters.time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = euler_integration(x0, v1, dt);

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
			energy.set_expression(Energy);
		}
	);

	//// Strain limiting
	sim.global_energy.add_energy("cloth_strain_limiting", this->conn_mesh_numbered_triangles,
		[&](symx::Energy& energy, symx::Element& mesh_idx_triangle)
		{
			// Unpack connectivity
			symx::Index tri_idx = mesh_idx_triangle[0];
			symx::Index mesh_idx = mesh_idx_triangle[1];
			std::vector<symx::Index> triangle = mesh_idx_triangle.slice(2, 5);

			// Create symbols
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, triangle);
			std::vector<symx::Vector> v1 = energy.make_vectors(this->model.v1, triangle);
			symx::Matrix DXinv = energy.make_matrix(this->DXinv, { 2, 2 }, tri_idx);
			symx::Scalar area = energy.make_scalar(this->triangle_area_rest, tri_idx);
			symx::Scalar strain_limiting_start = energy.make_scalar(this->strain_limiting_start, mesh_idx);
			symx::Scalar strain_limiting_stiffness = energy.make_scalar(this->strain_limiting_stiffness, mesh_idx);
			symx::Scalar dt = energy.make_scalar(sim.parameters.time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = euler_integration(x0, v1, dt);

			// Projection matrix
			symx::Vector v01 = x1[0] - x1[2];
			symx::Vector v02 = x1[1] - x1[2];
			symx::Vector px = v01.normalized();
			symx::Vector normal = v01.cross3(v02).normalized();
			symx::Vector py = normal.cross3(px);
			symx::Matrix P = symx::Matrix(symx::gather({ px, py }), { 2, 3 });

			// Projection and deformation gradient
			std::vector<symx::Vector> x1_ = { P * x1[0], P * x1[1], P * x1[2] };
			symx::Matrix Dx = symx::Matrix(symx::gather({
				x1_[1] - x1_[0],
				x1_[2] - x1_[0],
				}), { 2, 2 }).transpose();
			symx::Matrix F = Dx * DXinv;
			symx::Vector s = F.singular_values_2x2();
			symx::Scalar C = s[0] - strain_limiting_start;
			symx::Scalar E = area * strain_limiting_stiffness * C.powN(3);
			energy.set_conditional_expression(E, C);
			energy.activate(this->is_strain_limiting_active);
		}
	);

	//// Bergou06 Bending Energy
	sim.global_energy.add_energy("cloth_bending", this->conn_numbered_mesh_internal_edges,
		[&](symx::Energy& energy, symx::Element& mesh_idx_internal_edge)
		{
			// Unpack connectivity
			symx::Index ie_idx = mesh_idx_internal_edge[0];
			symx::Index mesh_idx = mesh_idx_internal_edge[1];
			std::vector<symx::Index> internal_edge = mesh_idx_internal_edge.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, internal_edge);
			std::vector<symx::Vector> v1 = energy.make_vectors(this->model.v1, internal_edge);
			symx::Matrix Q = energy.make_matrix(this->DXinv, { 4, 4 }, ie_idx);
			symx::Scalar bending_stiffness = energy.make_scalar(this->bending_stiffness, mesh_idx);
			symx::Scalar dt = energy.make_scalar(sim.parameters.time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = euler_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = dt.get_zero();
			for (int i = 0; i < 3; i++) {
				symx::Vector x = symx::Vector({ x1[0][i], x1[1][i], x1[2][i], x1[3][i] });
				E += x.transpose() * Q * x;
			}
			E *= bending_stiffness;
			energy.set_expression(E);
		}
	);

	//// Prescribed nodes
	sim.global_energy.add_energy("cloth_prescribed_positions", this->conn_enumerated_prescribed_positions,
		[&](symx::Energy& energy, symx::Element& node)
		{
			// Unpack connectivity
			symx::Index constraint_idx = node[0];
			symx::Index node_idx = node[1];

			//// Create symbols
			symx::Vector x0 = energy.make_vector(this->model.x0, node_idx);
			symx::Vector v1 = energy.make_vector(this->model.v1, node_idx);
			symx::Vector x1_prescribed = energy.make_vector(this->prescribed_positions, constraint_idx);
			symx::Scalar k = energy.make_scalar(sim.parameters.boundary_conditions_stiffness);
			symx::Scalar dt = energy.make_scalar(sim.parameters.time_step.value);

			// Time integration
			symx::Vector x1 = euler_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1 - x1_prescribed).squared_norm();
			energy.set_expression(E);
		}
	);

	//// Attachments
	sim.global_energy.add_energy("cloth_attachments", this->conn_attached_nodes,
		[&](symx::Energy& energy, symx::Element& node_pair)
		{
			//// Create symbols
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, node_pair);
			std::vector<symx::Vector> v1 = energy.make_vectors(this->model.v1, node_pair);
			symx::Scalar k = energy.make_scalar(sim.parameters.boundary_conditions_stiffness);
			symx::Scalar dt = energy.make_scalar(sim.parameters.time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = euler_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1[0] - x1[1]).squared_norm();
			energy.set_expression(E);
		}
	);

	/* -------------  Collisions ------------- */
	// Common symbol creation and manipulation functions ------------------------
	auto get_x1 = [&](std::vector<symx::Index> conn, symx::Energy& energy)
	{
		std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, conn);
		std::vector<symx::Vector> v1 = energy.make_vectors(this->model.v1, conn);
		symx::Scalar dt = energy.make_scalar(sim.parameters.time_step.value);
		return euler_integration(x0, v1, dt);
	};
	auto barrier_energy = [&](const symx::Scalar& d, const symx::Scalar& dhat, const symx::Scalar& k)
	{
		//symx::Scalar E = k * symx::log_barrier(d, dhat);
		return k * (dhat - d).powN(3);
	};
	auto set_barrier_energy = [&](const symx::Scalar& d, symx::Energy& energy)
	{
		symx::Scalar k = energy.make_scalar(sim.parameters.collision_stiffness.value);
		symx::Scalar dhat = energy.make_scalar(this->dhat);
		symx::Scalar E = barrier_energy(d, dhat, k);
		energy.set_expression(E);
		energy.activate(this->activate_collisions);
	};
	// -----------------------------------------------------------------------------
	
	// Point - Point
	sim.global_energy.add_energy("collision_cloth_cloth_point_point", this->contacts.point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> P = get_x1({ conn[0] }, energy);
			std::vector<symx::Vector> Q = get_x1({ conn[1] }, energy);
			symx::Scalar d = distance_point_point<symx::Scalar>(P[0], Q[0]);
			set_barrier_energy(d, energy);
		}
	);

	// Point - Edge
	sim.global_energy.add_energy("collision_cloth_cloth_point_edge", this->contacts.point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> P = get_x1({ conn[0] }, energy);
			std::vector<symx::Vector> Q = get_x1({ conn[1], conn[2] }, energy);
			symx::Scalar d = distance_point_line<symx::Scalar>(P[0], Q[0], Q[1]);
			set_barrier_energy(d, energy);
		}
	);

	// Point - Triangle
	sim.global_energy.add_energy("collision_cloth_cloth_point_triangle", this->contacts.point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> P = get_x1({ conn[0] }, energy);
			std::vector<symx::Vector> Q = get_x1({ conn[1], conn[2], conn[3] }, energy);
			symx::Scalar d = distance_point_plane<symx::Scalar>(P[0], Q[0], Q[1], Q[2]);
			set_barrier_energy(d, energy);
		}
	);

	// Edge - Edge
	sim.global_energy.add_energy("collision_cloth_cloth_edge_edge", this->contacts.edge_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Vector> P = get_x1({ conn[0], conn[1] }, energy);
			std::vector<symx::Vector> Q = get_x1({ conn[2], conn[3] }, energy);
			symx::Scalar d = distance_line_line<symx::Scalar>(P[0], P[1], Q[0], Q[1]);
			set_barrier_energy(d, energy);
		}
	);
}
void stark::models::Cloth::_before_time_step(Simulation& sim)
{
	Output::output->cout("Cloth::prepare_time_step()\n", 3);
	this->_init_simulation_structures(sim.options.n_threads);
	std::fill(this->model.v1.begin(), this->model.v1.end(), Eigen::Vector3d::Zero());
}
void stark::models::Cloth::_after_time_step(Simulation& sim)
{
	Output::output->cout("Cloth::postprocess_time_step()\n", 3);

	// Set final positions
	const double dt = sim.parameters.time_step.value;
	for (int i = 0; i < this->model.mesh.get_n_vertices(); i++) {
		this->model.x1[i] = this->model.x0[i] + dt * this->model.v1[i];
	}

	// x0 <- x1
	this->model.x0 = this->model.x1;
	this->model.v0 = this->model.v1;
}
void stark::models::Cloth::_write_frame(Simulation& sim)
{
	Output::output->cout("Cloth::write_frame()\n", 3);
	if (this->write_VTK) {
		utils::write_VTK(sim.output.get_vtk_path("cloth"), this->model.x1, this->model.mesh.connectivity);
	}
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
void stark::models::Cloth::set_damping(const double damping)
{
	this->damping = damping;
}
void stark::models::Cloth::set_material_preset(const int cloth_id, const MaterialPreset material)
{
	this->_exit_if_cloth_not_declared(cloth_id);
	switch (material)
	{
	case MaterialPreset::Cotton:
		this->set_density(cloth_id, 0.3);
		this->set_strain_parameters(cloth_id, 1e3, 0.3, 1.1);
		this->set_bending_stiffness(cloth_id, 1e-4);
		break;
	default:
		Output::output->cout("Error: cloth material preset not defined.");
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
void stark::models::Cloth::_exit_if_cloth_not_declared(const int cloth_id)
{
	Output::output->assert_with_exit(this->is_cloth_declared(cloth_id), "There is no cloth with id " + std::to_string(cloth_id) + " declared.");
}
void stark::models::Cloth::_init_simulation_structures(const int n_threads)
{
	Output::output->cout("Cloth::_init_simulation_structures()\n", 3);
	const auto& mesh_rest = this->model.mesh;

	// Inertia
	Output::output->cout("Inertia\n", 3);
	if (this->changed_discretization) {
		
		// Connectivity
		this->conn_nodes.resize(mesh_rest.get_n_vertices());
		std::iota(this->conn_nodes.begin(), this->conn_nodes.end(), 0);

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
	Output::output->cout("Strain\n", 3);
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

	// Bending
	Output::output->cout("Bending\n", 3);
	if (this->changed_discretization) {
		// Internal edges connectivity and Bergou06 matrix Q
		auto cotTheta = [](const Eigen::Vector3d& v, const Eigen::Vector3d& w) {
			const double cosTheta = v.dot(w);
			const double sinTheta = (v.cross(w)).norm();
			return (cosTheta / sinTheta);
		};

		std::vector<std::array<int, 4>> internal_angles;
		utils::find_internal_angles(internal_angles, mesh_rest.connectivity, mesh_rest.get_n_vertices());
		const int n = (int)internal_angles.size();
		this->conn_numbered_mesh_internal_edges.resize(n);
		this->bergou_Q_matrix.resize(n);
		this->edges = edges;

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
	Output::output->cout("Prescribed\n", 3);
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
	Output::output->cout("Attachments\n", 3);
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
void stark::models::Cloth::_update_collision_x(Simulation& sim)
{
	const double dt = sim.parameters.time_step.value;
	this->collision_x.resize(this->model.x0.size());
	for (int i = 0; i < this->model.mesh.get_n_vertices(); i++) {
		this->collision_x[i] = this->model.x0[i] + dt*this->model.v1[i];
	}
}
void stark::models::Cloth::_update_contacts(Simulation& sim)
{
	if (!this->activate_collisions) { return; }

	// Run CD
	this->_update_collision_x(sim);
	this->pd.clear();
	this->pd.add_mesh(&this->collision_x[0][0], (int)this->collision_x.size(), &this->model.mesh.connectivity[0][0], this->model.mesh.get_n_elements(), &this->edges[0][0], (int)this->edges.size());
	this->pd.disable_edge_edge(this->disable_edge_edge);
	this->pd.disable_point_triangle(this->disable_point_triangle);
	const tmcd::ProximityResults& proximity = this->pd.run(this->dhat);

	// Fill connectivities
	this->contacts.point_point.clear();
	this->contacts.point_edge.clear();
	this->contacts.point_triangle.clear();
	this->contacts.edge_edge.clear();
	for (const auto& pair : proximity.point_point) {
		this->contacts.point_point.push_back({pair.first.idx, pair.second.idx});
	}
	for (const auto& pair : proximity.point_edge) {
		this->contacts.point_edge.push_back({ pair.first.idx, pair.second.vertices[0], pair.second.vertices[1] });
	}
	for (const auto& pair : proximity.point_triangle) {
		this->contacts.point_triangle.push_back({ pair.first.idx, pair.second.vertices[0], pair.second.vertices[1], pair.second.vertices[2] });
	}
	for (const auto& pair : proximity.edge_edge) {
		this->contacts.edge_edge.push_back({ pair.first.vertices[0], pair.first.vertices[1], pair.second.vertices[0], pair.second.vertices[1] });
	}
}
bool stark::models::Cloth::_is_valid_configuration(Simulation& sim)
{
	this->_update_collision_x(sim);
	this->id.clear();
	this->id.add_mesh(&this->collision_x[0][0], (int)this->collision_x.size(), &this->model.mesh.connectivity[0][0], this->model.mesh.get_n_elements(), &this->edges[0][0], (int)this->edges.size());
	const tmcd::IntersectionResults& intersections = this->id.run();
	return intersections.edge_triangle.size() == 0;
}
bool stark::models::Cloth::is_cloth_declared(const int cloth_id) const
{
	return cloth_id < this->get_n_cloths();
}
int stark::models::Cloth::get_n_cloths() const
{
	return this->model.mesh.get_n_meshes();
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
