#include "DeformableSolids.h"

#include "../utils/mesh_utils.h"
#include "time_integration.h"

#include <vtkio>


void stark::models::DeformableSolids::init(Stark& sim)
{
	this->_init_simulation_structures(sim.settings.execution.n_threads);

	// DoFs
	this->dof = sim.global_energy.add_dof_array(this->model.v1, "ds_v1");

	// Callbacks
	sim.callbacks.before_time_step.push_back([&]() { this->_before_time_step(sim); });
	sim.callbacks.after_time_step.push_back([&]() { this->_after_time_step(sim); });
	sim.callbacks.write_frame.push_back([&]() { this->_write_frame(sim); });

	// Energy declarations
	this->_energies_mechanical(sim);
}
void stark::models::DeformableSolids::set_vertex_target_position_as_initial(const int body_id, const int vertex_id)
{
	this->set_vertex_target_position(body_id, vertex_id, this->model.mesh.vertices[this->model.mesh.get_global_vertex_idx(body_id, vertex_id)]);
}
void stark::models::DeformableSolids::set_vertex_target_position(const int body_id, const int vertex_id, const Eigen::Vector3d& position)
{
	this->prescribed_nodes_map[this->model.mesh.get_global_vertex_idx(body_id, vertex_id)] = position;
	this->changed_prescribed_vertices = true;
}
void stark::models::DeformableSolids::freeze(const int body_id)
{
	for (int i = 0; i < this->model.mesh.get_n_vertices(body_id); i++) {
		const int idx = this->model.mesh.get_global_vertex_idx(body_id, i);
		this->set_vertex_target_position(body_id, i, this->model.x1[idx]);
	}
}
void stark::models::DeformableSolids::clear_vertex_target_position()
{
	this->prescribed_nodes_map.clear();
	this->changed_prescribed_vertices = true;
}
void stark::models::DeformableSolids::set_acceleration(const int body_id, const int vertex_idx, const Eigen::Vector3d& acceleration)
{
	this->model.set_acceleration(body_id, vertex_idx, acceleration);
}
void stark::models::DeformableSolids::add_acceleration(const int body_id, const int vertex_idx, const Eigen::Vector3d& acceleration)
{
	this->model.add_acceleration(body_id, vertex_idx, acceleration);
}
void stark::models::DeformableSolids::set_velocity(const int body_id, const int vertex_idx, const Eigen::Vector3d& velocity)
{
	this->model.set_velocity(body_id, vertex_idx, velocity);
}
void stark::models::DeformableSolids::add_velocity(const int body_id, const int vertex_idx, const Eigen::Vector3d& velocity)
{
	this->model.add_velocity(body_id, vertex_idx, velocity);
}
void stark::models::DeformableSolids::set_position(const int body_id, const int vertex_idx, const Eigen::Vector3d& position)
{
	this->model.set_position(body_id, vertex_idx, position);
}
void stark::models::DeformableSolids::add_displacement(const int body_id, const int vertex_idx, const Eigen::Vector3d& displacement)
{
	this->model.add_displacement(body_id, vertex_idx, displacement);
}
void stark::models::DeformableSolids::clear_acceleration()
{
	std::fill(this->model.a.begin(), this->model.a.end(), Eigen::Vector3d::Zero());
}
int stark::models::DeformableSolids::add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets)
{
	const int body_id = this->get_n_bodies();
	this->model.add_mesh(vertices, tets);

	// Add per-mesh parameters
	//// Inertia
	this->density.push_back(1000.0);

	//// Strain
	this->young_modulus.push_back(1e5);
	this->poisson_ratio.push_back(0.3);

	this->changed_discretization = true;

	return body_id;
}
void stark::models::DeformableSolids::set_damping(const double inertial_damping)
{
	this->inertial_damping = inertial_damping;
}
void stark::models::DeformableSolids::set_density(const int body_id, const double density)
{
	this->density[body_id] = density;
	this->changed_discretization = true;
}
void stark::models::DeformableSolids::set_strain_parameters(const int body_id, const double young_modulus, const double poisson_ratio)
{
	this->young_modulus[body_id] = young_modulus;
	this->poisson_ratio[body_id] = poisson_ratio;
}
int stark::models::DeformableSolids::get_n_bodies() const
{
	return this->model.mesh.get_n_meshes();
}
bool stark::models::DeformableSolids::is_empty() const
{
	return this->get_n_bodies() == 0;
}
Eigen::Vector3d stark::models::DeformableSolids::get_vertex(const int body_id, const int vertex_idx) const
{
	return this->model.x1[this->model.mesh.get_global_vertex_idx(body_id, vertex_idx)];
}
Eigen::Vector3d stark::models::DeformableSolids::get_velocity(const int body_id, const int vertex_idx) const
{
	return this->model.v1[this->model.mesh.get_global_vertex_idx(body_id, vertex_idx)];
}

void stark::models::DeformableSolids::_init_simulation_structures(const int n_threads)
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
		for (int mesh_i = 0; mesh_i < this->get_n_bodies(); mesh_i++) {
			const double density = this->density[mesh_i];
			mesh_rest.for_each_element_parallel_const(mesh_i,
				[&](const int tet_glob_idx, const std::array<int, 4>& tet_glob, std::array<const Eigen::Vector3d*, 4>& vertices)
				{
					const double lumped_mass = density*utils::unsigned_tetra_volume(*vertices[0], *vertices[1], *vertices[2], *vertices[3])/4.0;
					this->lumped_mass[tet_glob[0]] += lumped_mass;
					this->lumped_mass[tet_glob[1]] += lumped_mass;
					this->lumped_mass[tet_glob[2]] += lumped_mass;
					this->lumped_mass[tet_glob[3]] += lumped_mass;
				}, n_threads
			);
		}
	}

	// Strain
	if (this->changed_discretization) {
		this->conn_mesh_numbered_tets.resize(mesh_rest.get_n_elements());
		for (int mesh_i = 0; mesh_i < this->get_n_bodies(); mesh_i++) {
			mesh_rest.for_each_element_parallel_const(mesh_i,
				[&](const int tet_glob_idx, const std::array<int, 4>& tet_glob, std::array<const Eigen::Vector3d*, 4>& vertices)
				{
					this->conn_mesh_numbered_tets[tet_glob_idx] = { tet_glob_idx, mesh_i, tet_glob[0], tet_glob[1], tet_glob[2], tet_glob[3]  };
				}, n_threads
			);
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

	// Reset
	this->changed_discretization = false;
	this->changed_prescribed_vertices = false;
}

void stark::models::DeformableSolids::_before_time_step(Stark& sim)
{
	if (this->is_empty()) { return; }

	// Reset simulation structures if anything changed
	this->_init_simulation_structures(sim.settings.execution.n_threads);

	// Set next time velocities estimation to zero to avoid invalid state outside of the minimzer
	std::fill(this->model.v1.begin(), this->model.v1.end(), Eigen::Vector3d::Zero());
}
void stark::models::DeformableSolids::_after_time_step(Stark& sim)
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
void stark::models::DeformableSolids::_write_frame(Stark& sim)
{
	if (this->is_empty()) { return; }

	const std::string path = sim.get_vtk_path("deformable_solids");
	vtkio::VTKFile vtk_file;
	if (this->model.x1.size() == 0) {
		vtk_file.write_empty(path);
	}
	else {
		vtk_file.set_points_from_twice_indexable(this->model.x1);
		vtk_file.set_cells_from_twice_indexable(this->model.mesh.connectivity, vtkio::CellType::Tetra);
		vtk_file.write(path);
	}
}

void stark::models::DeformableSolids::_energies_mechanical(Stark& sim)
{
	// Lumped mass inertia
	sim.global_energy.add_energy("deformable_solids_inertia", this->conn_nodes,
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
	sim.global_energy.add_energy("deformable_solids_strain", this->conn_mesh_numbered_tets,
		[&](symx::Energy& energy, symx::Element& mesh_idx_triangle)
		{
			// Unpack connectivity
			symx::Index tet_idx = mesh_idx_triangle[0];
			symx::Index mesh_idx = mesh_idx_triangle[1];
			std::vector<symx::Index> tet = mesh_idx_triangle.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> X = energy.make_vectors(this->model.X, tet);
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(dof, this->model.v1, tet);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->model.x0, tet);
			symx::Scalar E = energy.make_scalar(this->young_modulus, mesh_idx);
			symx::Scalar nu = energy.make_scalar(this->poisson_ratio, mesh_idx);
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Kinematics
			auto jacobian = [](const std::vector<symx::Vector>& xh)
			{
				return symx::Matrix(
					{
						-xh[0][0] + xh[1][0], -xh[0][0] + xh[2][0], -xh[0][0] + xh[3][0],
						-xh[0][1] + xh[1][1], -xh[0][1] + xh[2][1], -xh[0][1] + xh[3][1],
						-xh[0][2] + xh[1][2], -xh[0][2] + xh[2][2], -xh[0][2] + xh[3][2]
					},
					{ 3, 3 }
				);
			};
			symx::Matrix J = jacobian(X);
			symx::Matrix j1 = jacobian(x1);
			symx::Matrix F = j1 * J.inv();

			// Constitutive model
			symx::Scalar mu = E / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (E * nu) / ((1.0 + nu) * (1.0 - 2.0 * nu)); // 3D

			// [Smith et al. 2022] Stable Neo-Hookean Flesh Simulation
			// Eq. 49 from [Smith et al. 2022]
			//symx::Scalar mu_ = 4.0/3.0*mu;
			//symx::Scalar lambda_ = lambda + 5.0/6.0*mu;
			//symx::Scalar detF = F.det();
			//symx::Scalar Ic = F.frobenius_norm_sq();
			//symx::Scalar alpha = 1.0 + mu_ / lambda_ - mu_ / (4.0 * lambda_);
			//symx::Scalar energy_density = 0.5*mu_*(Ic - 3.0) + 0.5*lambda_*(detF - alpha).powN(2) - 0.5*mu_*symx::log(Ic + 1.0);

			// Neo-Hookean
			symx::Scalar detF = F.det();
			symx::Scalar Ic = F.frobenius_norm_sq();
			symx::Scalar logJ = symx::log(detF);
			symx::Scalar energy_density = 0.5*mu*(Ic - 3.0) - mu*logJ + 0.5*lambda*logJ.powN(2);
			
			// Total energy for a linear tet
			const double w = 1.0 / 6.0;
			symx::Scalar Energy = energy_density * J.det() * w;
			energy.set(Energy);
		}
	);
	
	// Prescribed nodes
	sim.global_energy.add_energy("deformable_solids_prescribed_positions", this->conn_enumerated_prescribed_positions,
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
}
