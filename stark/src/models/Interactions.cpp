#include "Interactions.h"

#include "distances.h"
#include "time_integration.h"
#include "rigidbody_transformations.h"


void stark::models::Interactions::init(Stark& sim, Cloth* cloth, RigidBodies* rigid_bodies)
{
	// Keep a pointer to the physical systems
	this->cloth = cloth;
	this->rigid_bodies = rigid_bodies;

	// Callbacks
	//sim.callbacks.before_time_step.push_back([&]() { this->_before_time_step(sim); });
	sim.callbacks.before_energy_evaluation.push_back([&]() { this->_update_contacts(sim); });
	sim.callbacks.is_state_valid.push_back([&]() { return this->_is_valid_configuration(sim); });

	// Energy declarations
	this->_energies_contact(sim);
}

bool stark::models::Interactions::is_empty() const
{
	return this->cloth->is_empty() && this->rigid_bodies->is_empty();
}

const tmcd::ProximityResults& stark::models::Interactions::_run_proximity_detection(const std::vector<Eigen::Vector3d>& x_cloth, const std::vector<Eigen::Vector3d>& x_rb, Stark& sim)
{
	this->pd.clear();
	this->pd.set_n_threads(sim.settings.execution.n_threads);
	this->pd.set_edge_edge_parallel_cutoff(sim.settings.contact.edge_edge_cross_norm_sq_cutoff);
	this->pd.activate_point_triangle(sim.settings.contact.triangle_point_enabled);
	this->pd.activate_edge_edge(sim.settings.contact.edge_edge_enabled);
	if (!this->cloth->is_empty()) {
		this->cloth_id = this->pd.add_mesh(&x_cloth[0][0], (int)x_cloth.size(), &this->cloth->model.mesh.connectivity[0][0], this->cloth->model.mesh.get_n_elements(), &this->cloth->edges.connectivity[0][0], (int)this->cloth->edges.get_n_edges());
		this->pd.add_blacklist(this->cloth_id, this->cloth_id);
	}
	if (!this->rigid_bodies->is_empty()) {
		this->rigid_bodies_id = this->pd.add_mesh(&x_rb[0][0], (int)x_rb.size(), &this->rigid_bodies->mesh.connectivity[0][0], this->rigid_bodies->mesh.get_n_elements(), &this->rigid_bodies->edges.connectivity[0][0], (int)this->rigid_bodies->edges.get_n_edges());
		this->pd.add_blacklist(this->rigid_bodies_id, this->rigid_bodies_id);
	}

	const tmcd::ProximityResults& proximity = this->pd.run(sim.settings.contact.dhat);
	return proximity;
}

void stark::models::Interactions::_update_contacts(Stark& sim)
{
	if (this->is_empty()) { return; }
	if (!sim.settings.contact.collisions_enabled) { return; }

	// Proximity detection
	// this->_update_collision_x1(sim);  ->  collision_x1 should be updated given the order of lambda insertions
	const tmcd::ProximityResults& proximity = this->_run_proximity_detection(this->cloth->collision_x1, this->rigid_bodies->collision_x1, sim);

	// Fill connectivities
	this->contacts.clear();

	//// Point - Triangle
	for (const auto& pair : proximity.point_triangle.point_point) {
		const tmcd::Point& p = pair.first;
		const tmcd::TrianglePoint& tp = pair.second;
		const tmcd::Point& q = tp.point;

		if (p.set == this->rigid_bodies_id && q.set == this->cloth_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(p.idx);
			this->contacts.point_triangle.rb_d_point_point.push_back({ rb_idx, p.idx, q.idx });
		}
		else if (p.set == this->cloth_id && q.set == this->rigid_bodies_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(q.idx);
			this->contacts.point_triangle.rb_d_point_point.push_back({ rb_idx, q.idx, p.idx });
		}
		else {
			exit(9);
		}
	}
	for (const auto& pair : proximity.point_triangle.point_edge) {
		const tmcd::Point& point = pair.first;
		const tmcd::TriangleEdge& te = pair.second;
		const tmcd::TriangleEdge::Edge& edge = te.edge;

		if (point.set == this->rigid_bodies_id && edge.set == this->cloth_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(point.idx);
			this->contacts.point_triangle.rb_d_point_edge.push_back({ rb_idx, point.idx, edge.vertices[0], edge.vertices[1] });
		}
		else if (point.set == this->cloth_id && edge.set == this->rigid_bodies_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(edge.vertices[0]);
			this->contacts.point_triangle.rb_d_edge_point.push_back({ rb_idx, edge.vertices[0], edge.vertices[1], point.idx });
		}
		else {
			exit(9);
		}
	}
	for (const auto& pair : proximity.point_triangle.point_triangle) {
		const tmcd::Point& point = pair.first;
		const tmcd::Triangle& triangle = pair.second;

		if (point.set == this->rigid_bodies_id && triangle.set == this->cloth_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(point.idx);
			this->contacts.point_triangle.rb_d_point_triangle.push_back({ rb_idx, point.idx, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2] });
		}
		else if (point.set == this->cloth_id && triangle.set == this->rigid_bodies_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(triangle.vertices[0]);
			this->contacts.point_triangle.rb_d_triangle_point.push_back({ rb_idx, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2], point.idx });
		}
		else {
			exit(9);
		}
	}
}
bool stark::models::Interactions::_is_valid_configuration(Stark& sim)
{
	if (this->is_empty()) { return true; }
	if (!sim.settings.contact.collisions_enabled) { return true; }
	if (!sim.settings.contact.enable_intersection_test) { return true; }

	// this->_update_collision_x1(sim, sim.settings.simulation.adaptive_time_step.value);  ->  collision_x1 should be updated given the order of lambda insertions
	this->id.clear();
	this->id.set_n_threads(sim.settings.execution.n_threads);
	const int cloth = this->id.add_mesh(&this->cloth->collision_x1[0][0], (int)this->cloth->collision_x1.size(), &this->cloth->model.mesh.connectivity[0][0], this->cloth->model.mesh.get_n_elements(), &this->cloth->edges.connectivity[0][0], (int)this->cloth->edges.get_n_edges());
	const int rb = this->id.add_mesh(&this->rigid_bodies->collision_x1[0][0], (int)this->rigid_bodies->collision_x1.size(), &this->rigid_bodies->mesh.connectivity[0][0], this->rigid_bodies->mesh.get_n_elements(), &this->rigid_bodies->edges.connectivity[0][0], (int)this->rigid_bodies->edges.get_n_edges());

	this->id.add_blacklist(cloth, cloth);
	this->id.add_blacklist(rb, rb);

	const tmcd::IntersectionResults& intersections = this->id.run();
	return intersections.edge_triangle.size() == 0;
}

void stark::models::Interactions::_energies_contact(Stark& sim)
{
	// Common symbol creation and manipulation functions ------------------------
	auto get_rb_x1 = [&](const std::vector<symx::Index>& indices, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Vector v1 = energy.make_dof_vector(this->rigid_bodies->dof_v, this->rigid_bodies->v1, rb_idx);
		symx::Vector w1 = energy.make_dof_vector(this->rigid_bodies->dof_w, this->rigid_bodies->w1, rb_idx);
		symx::Vector t0 = energy.make_vector(this->rigid_bodies->t0, rb_idx);
		symx::Vector q0 = energy.make_vector(this->rigid_bodies->q0_, rb_idx);

		symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
		symx::Vector t1 = time_integration(t0, v1, dt);

		std::vector<symx::Vector> x_loc = energy.make_vectors(this->rigid_bodies->mesh.vertices, indices);

		std::vector<symx::Vector> x1;
		for (const symx::Vector& x_loc_a : x_loc) {
			x1.push_back(local_to_global_point(x_loc_a, t1, R1));
		}
		return x1;
	};
	auto get_d_x1 = [&](const std::vector<symx::Index>& conn, const symx::Scalar& dt, symx::Energy& energy)
	{
		std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->cloth->dof, this->cloth->model.v1, conn);
		std::vector<symx::Vector> x0 = energy.make_vectors(this->cloth->model.x0, conn);
		return time_integration(x0, v1, dt);
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
		energy.activate(sim.settings.contact.collisions_enabled);
	};
	// -----------------------------------------------------------------------------

	// Point - Triangle
	//// RB -> D: Point - Point
	sim.global_energy.add_energy("collision_rb_d_pt_point_point", this->contacts.point_triangle.rb_d_point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> P = get_rb_x1({ conn["rb_p_loc"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> Q = get_d_x1({ conn["q"] }, dt, energy);
			symx::Scalar d = distance_point_point(P[0], Q[0]);
			set_barrier_energy(d, energy);
		}
	);

	//// RB -> D: Point - Edge
	sim.global_energy.add_energy("collision_rb_d_pt_point_edge", this->contacts.point_triangle.rb_d_point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> P = get_rb_x1({ conn["rb_p_loc"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> Q = get_d_x1({ conn["e0"], conn["e1"] }, dt, energy);
			symx::Scalar d = distance_point_line(P[0], Q[0], Q[1]);
			set_barrier_energy(d, energy);
		}
	);

	//// RB -> D: Point - Triangle
	sim.global_energy.add_energy("collision_rb_d_pt_point_triangle", this->contacts.point_triangle.rb_d_point_triangle,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> P = get_rb_x1({ conn["rb_p_loc"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> Q = get_d_x1({ conn["t0"], conn["t1"], conn["t2"] }, dt, energy);
			symx::Scalar d = distance_point_plane(P[0], Q[0], Q[1], Q[2]);
			set_barrier_energy(d, energy);
		}
	);
	
	//// D -> RB: Point - Edge
	sim.global_energy.add_energy("collision_rb_d_pt_edge_point", this->contacts.point_triangle.rb_d_edge_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> P = get_rb_x1({ conn["rb_e0_loc"], conn["rb_e1_loc"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> Q = get_d_x1({ conn["p"] }, dt, energy);
			symx::Scalar d = distance_point_line(Q[0], P[0], P[1]);
			set_barrier_energy(d, energy);
		}
	);

	//// D -> RB: Point - Triangle
	sim.global_energy.add_energy("collision_rb_d_pt_triangle_point", this->contacts.point_triangle.rb_d_triangle_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> P = get_rb_x1({ conn["rb_t0_loc"], conn["rb_t1_loc"], conn["rb_t2_loc"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> Q = get_d_x1({ conn["p"] }, dt, energy);
			symx::Scalar d = distance_point_plane(Q[0], P[0], P[1], P[2]);
			set_barrier_energy(d, energy);
		}
	);
}
