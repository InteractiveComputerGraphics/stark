#include "Interactions.h"

#include "distances.h"
#include "time_integration.h"
#include "friction_geometry.h"
#include "rigidbody_transformations.h"


void stark::models::Interactions::init(Stark& sim, Cloth* cloth, RigidBodies* rigid_bodies)
{
	// Keep a pointer to the physical systems
	this->cloth = cloth;
	this->rigid_bodies = rigid_bodies;

	// Callbacks
	sim.callbacks.before_time_step.push_back([&]() { this->_update_friction_contacts(sim); });
	sim.callbacks.before_energy_evaluation.push_back([&]() { this->_update_contacts(sim); });
	sim.callbacks.is_state_valid.push_back([&]() { return this->_is_valid_configuration(sim); });

	// Energy declarations
	this->_energies_contact(sim);
	this->_energies_friction(sim);
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

	//// Edge - Edge
	for (const auto& pair : proximity.edge_edge.point_point) {
		const tmcd::EdgePoint& ep_a = pair.first;
		const tmcd::EdgePoint& ep_b = pair.second;

		if (ep_a.point.set == this->rigid_bodies_id && ep_b.point.set == this->cloth_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(ep_a.point.idx);
			this->contacts.edge_edge.rb_d_point_point.push_back({ rb_idx, ep_a.edge.vertices[0], ep_a.edge.vertices[1], ep_a.point.idx, ep_b.edge.vertices[0], ep_b.edge.vertices[1], ep_b.point.idx });
		}
		else if (ep_a.point.set == this->cloth_id && ep_b.point.set == this->rigid_bodies_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(ep_b.point.idx);
			this->contacts.edge_edge.rb_d_point_point.push_back({ rb_idx, ep_b.edge.vertices[0], ep_b.edge.vertices[1], ep_b.point.idx, ep_a.edge.vertices[0], ep_a.edge.vertices[1], ep_a.point.idx });
		}
		else {
			exit(9);
		}
	}
	for (const auto& pair : proximity.edge_edge.point_edge) {
		const tmcd::EdgePoint& ep_a = pair.first;
		const tmcd::Edge& e_b = pair.second;

		if (ep_a.point.set == this->rigid_bodies_id && e_b.set == this->cloth_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(ep_a.point.idx);
			this->contacts.edge_edge.rb_d_point_edge.push_back({ rb_idx, ep_a.edge.vertices[0], ep_a.edge.vertices[1], ep_a.point.idx, e_b.vertices[0], e_b.vertices[1] });
		}
		else if (ep_a.point.set == this->cloth_id && e_b.set == this->rigid_bodies_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(e_b.vertices[0]);
			this->contacts.edge_edge.rb_d_edge_point.push_back({ rb_idx, e_b.vertices[0], e_b.vertices[1], ep_a.edge.vertices[0], ep_a.edge.vertices[1], ep_a.point.idx });
		}
		else {
			exit(9);
		}
	}
	for (const auto& pair : proximity.edge_edge.edge_edge) {
		if (pair.first.set == this->rigid_bodies_id && pair.second.set == this->cloth_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(pair.first.vertices[0]);
			this->contacts.edge_edge.rb_d_edge_edge.push_back({ rb_idx, pair.first.vertices[0], pair.first.vertices[1], pair.second.vertices[0], pair.second.vertices[1] });
		}
		else if (pair.first.set == this->cloth_id && pair.second.set == this->rigid_bodies_id) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(pair.second.vertices[0]);
			this->contacts.edge_edge.rb_d_edge_edge.push_back({ rb_idx, pair.second.vertices[0], pair.second.vertices[1], pair.first.vertices[0], pair.first.vertices[1] });
		}
		else {
			exit(9);
		}
	}
}
void stark::models::Interactions::_update_friction_contacts(Stark& sim)
{
	if (!sim.settings.contact.collisions_enabled) { return; }
	if (!sim.settings.contact.friction_enabled) { return; }

	// Proximity detection
	// this->_update_collision_x1(sim, /* dt = */0.0);  ->  collision_x1 should be updated given the order of lambda insertions
	const std::vector<Eigen::Vector3d>& x_cloth = this->cloth->model.x0;
	const std::vector<Eigen::Vector3d>& x_rb = this->rigid_bodies->collision_x1;
	const tmcd::ProximityResults& proximity = this->_run_proximity_detection(x_cloth, x_rb, sim);

	// Friction force
	const double k = sim.settings.contact.adaptive_contact_stiffness.value;
	const double dhat = sim.settings.contact.dhat;
	auto force = [&](double d) { return k * 3.0 * std::pow(dhat - d, 2); };

	// Fill friction data
	this->friction.clear();
	const std::vector<double>& mu_cloth = this->cloth->vertex_mu;
	const std::vector<double>& mu_rb = this->rigid_bodies->mu;

	// Point - Triangle
	//// Point - Point
	for (const auto& pair : proximity.point_triangle.point_point) {
		const tmcd::Point& p = pair.first;
		const tmcd::TrianglePoint& tp = pair.second;
		const tmcd::Point& q = tp.point;

		const bool is_p_rb = (p.set == this->rigid_bodies_id) && (q.set == this->cloth_id);
		const int p_idx = (is_p_rb) ? p.idx : q.idx;
		const int q_idx = (is_p_rb) ? q.idx : p.idx;

		const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(p_idx);
		const double mu = 0.5 * (mu_rb[rb_idx] + mu_cloth[q_idx]);
		if (mu > 0.0) {
			this->friction.rb_d_point_point.conn.numbered_push_back({ rb_idx, p_idx, q_idx });
			this->friction.rb_d_point_point.contact.T.push_back(projection_matrix_point_point(x_rb[p_idx], x_cloth[q_idx]));
			this->friction.rb_d_point_point.contact.mu.push_back(mu);
			this->friction.rb_d_point_point.contact.fn.push_back(force(pair.distance));
		}
	}
	//// Point - Edge
	for (const auto& pair : proximity.point_triangle.point_edge) {
		const tmcd::Point& p = pair.first;
		const tmcd::TriangleEdge& te = pair.second;
		const tmcd::TriangleEdge::Edge& edge = te.edge;

		if ((p.set == this->rigid_bodies_id) && (edge.set == this->cloth_id)) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(p.idx);
			const double mu = 0.5 * (mu_rb[rb_idx] + mu_cloth[edge.vertices[0]]);
			if (mu > 0.0) {
				this->friction.rb_d_point_edge.conn.numbered_push_back({ rb_idx, p.idx, edge.vertices[0], edge.vertices[1] });
				this->friction.rb_d_point_edge.bary.push_back(barycentric_point_edge(x_rb[p.idx], x_cloth[edge.vertices[0]], x_cloth[edge.vertices[1]]));
				this->friction.rb_d_point_edge.contact.T.push_back(projection_matrix_point_edge(x_rb[p.idx], x_cloth[edge.vertices[0]], x_cloth[edge.vertices[1]]));
				this->friction.rb_d_point_edge.contact.mu.push_back(mu);
				this->friction.rb_d_point_edge.contact.fn.push_back(force(pair.distance));
			}
		}
		else {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(edge.vertices[0]);
			const double mu = 0.5 * (mu_rb[rb_idx] + mu_cloth[p.idx]);
			if (mu > 0.0) {
				this->friction.rb_d_edge_point.conn.numbered_push_back({ rb_idx, edge.vertices[0], edge.vertices[1], p.idx });
				this->friction.rb_d_edge_point.bary.push_back(barycentric_point_edge(x_cloth[p.idx], x_rb[edge.vertices[0]], x_rb[edge.vertices[1]]));
				this->friction.rb_d_edge_point.contact.T.push_back(projection_matrix_point_edge(x_cloth[p.idx], x_rb[edge.vertices[0]], x_rb[edge.vertices[1]]));
				this->friction.rb_d_edge_point.contact.mu.push_back(mu);
				this->friction.rb_d_edge_point.contact.fn.push_back(force(pair.distance));
			}
		}
	}
	//// Point - Triangle
	for (const auto& pair : proximity.point_triangle.point_triangle) {
		const tmcd::Point& p = pair.first;
		const tmcd::Triangle& t = pair.second;

		if ((p.set == this->rigid_bodies_id) && (t.set == this->cloth_id)) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(p.idx);
			const double mu = 0.5 * (mu_rb[rb_idx] + mu_cloth[t.vertices[0]]);
			if (mu > 0.0) {
				this->friction.rb_d_point_triangle.conn.numbered_push_back({ rb_idx, p.idx, t.vertices[0], t.vertices[1], t.vertices[2] });
				this->friction.rb_d_point_triangle.bary.push_back(barycentric_point_triangle(x_rb[p.idx], x_cloth[t.vertices[0]], x_cloth[t.vertices[1]], x_cloth[t.vertices[2]]));
				this->friction.rb_d_point_triangle.contact.T.push_back(projection_matrix_triangle(x_cloth[t.vertices[0]], x_cloth[t.vertices[1]], x_cloth[t.vertices[2]]));
				this->friction.rb_d_point_triangle.contact.mu.push_back(mu);
				this->friction.rb_d_point_triangle.contact.fn.push_back(force(pair.distance));
			}
		}
		else {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(t.vertices[0]);
			const double mu = 0.5 * (mu_rb[rb_idx] + mu_cloth[p.idx]);
			if (mu > 0.0) {
				this->friction.rb_d_triangle_point.conn.numbered_push_back({ rb_idx, t.vertices[0], t.vertices[1], t.vertices[2], p.idx });
				this->friction.rb_d_triangle_point.bary.push_back(barycentric_point_triangle(x_cloth[p.idx], x_rb[t.vertices[0]], x_rb[t.vertices[1]], x_rb[t.vertices[2]]));
				this->friction.rb_d_triangle_point.contact.T.push_back(projection_matrix_triangle(x_rb[t.vertices[0]], x_rb[t.vertices[1]], x_rb[t.vertices[2]]));
				this->friction.rb_d_triangle_point.contact.mu.push_back(mu);
				this->friction.rb_d_triangle_point.contact.fn.push_back(force(pair.distance));
			}
		}
	}

	// Edge - Edge
	const std::vector<Eigen::Vector3d>& X_rb = this->rigid_bodies->mesh.vertices;
	const std::vector<Eigen::Vector3d>& X_cloth = this->cloth->model.X;
	auto are_not_almost_parallel = [&](const tmcd::Edge& rb_edge, const tmcd::Edge& cloth_edge)
	{
		const Eigen::Vector3d& ea0 = x_rb[rb_edge.vertices[0]];
		const Eigen::Vector3d& ea1 = x_rb[rb_edge.vertices[1]];
		const Eigen::Vector3d& eb0 = x_cloth[cloth_edge.vertices[0]];
		const Eigen::Vector3d& eb1 = x_cloth[cloth_edge.vertices[1]];
		return (ea1 - ea0).cross(eb1 - eb0).squaredNorm() > 1e-16;
	};
	auto mollifier_f = [&](const tmcd::Edge& rb_edge, const tmcd::Edge& cloth_edge)
	{
		return edge_edge_mollifier(x_rb[rb_edge.vertices[0]], x_rb[rb_edge.vertices[1]], x_cloth[cloth_edge.vertices[0]], x_cloth[cloth_edge.vertices[1]],
			X_rb[rb_edge.vertices[0]], X_rb[rb_edge.vertices[1]], X_cloth[cloth_edge.vertices[0]], X_cloth[cloth_edge.vertices[1]]);
	};

	//// Point - Point
	for (const auto& pair : proximity.edge_edge.point_point) {
		const tmcd::EdgePoint& ep_a = pair.first;
		const tmcd::EdgePoint& ep_b = pair.second;
		const tmcd::Point& p = ep_a.point;
		const tmcd::Point& q = ep_b.point;

		const bool is_p_rb = (p.set == this->rigid_bodies_id) && (q.set == this->cloth_id);
		const int p_idx = (is_p_rb) ? p.idx : q.idx;
		const int q_idx = (is_p_rb) ? q.idx : p.idx;

		const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(p_idx);
		const double mu = 0.5 * (mu_rb[rb_idx] + mu_cloth[q_idx]);
		const bool not_parallel = (is_p_rb) ? are_not_almost_parallel(ep_a.edge, ep_b.edge) : are_not_almost_parallel(ep_b.edge, ep_a.edge);
		if (mu > 0.0 && not_parallel) {
			const double mollifier = (is_p_rb) ? mollifier_f(ep_a.edge, ep_b.edge) : mollifier_f(ep_b.edge, ep_a.edge);
			this->friction.rb_d_point_point.conn.numbered_push_back({ rb_idx, p_idx, q_idx });
			this->friction.rb_d_point_point.contact.T.push_back(projection_matrix_point_point(x_rb[p_idx], x_cloth[q_idx]));
			this->friction.rb_d_point_point.contact.mu.push_back(mu);
			this->friction.rb_d_point_point.contact.fn.push_back(mollifier*force(pair.distance));
		}
	}
	//// Point - Edge
	for (const auto& pair : proximity.edge_edge.point_edge) {
		const tmcd::EdgePoint& ep = pair.first;
		const tmcd::Point& p = ep.point;
		const tmcd::Edge& edge = pair.second;

		if ((p.set == this->rigid_bodies_id) && (edge.set == this->cloth_id)) {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(p.idx);
			const double mu = 0.5 * (mu_rb[rb_idx] + mu_cloth[edge.vertices[0]]);
			if (mu > 0.0 && are_not_almost_parallel(ep.edge, edge)) {
				this->friction.rb_d_point_edge.conn.numbered_push_back({ rb_idx, p.idx, edge.vertices[0], edge.vertices[1] });
				this->friction.rb_d_point_edge.bary.push_back(barycentric_point_edge(x_rb[p.idx], x_cloth[edge.vertices[0]], x_cloth[edge.vertices[1]]));
				this->friction.rb_d_point_edge.contact.T.push_back(projection_matrix_point_edge(x_rb[p.idx], x_cloth[edge.vertices[0]], x_cloth[edge.vertices[1]]));
				this->friction.rb_d_point_edge.contact.mu.push_back(mu);
				this->friction.rb_d_point_edge.contact.fn.push_back(mollifier_f(ep.edge, edge)*force(pair.distance));
			}
		}
		else {
			const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(edge.vertices[0]);
			const double mu = 0.5 * (mu_rb[rb_idx] + mu_cloth[p.idx]);
			if (mu > 0.0 && are_not_almost_parallel(edge, ep.edge)) {
				this->friction.rb_d_edge_point.conn.numbered_push_back({ rb_idx, edge.vertices[0], edge.vertices[1], p.idx });
				this->friction.rb_d_edge_point.bary.push_back(barycentric_point_edge(x_cloth[p.idx], x_rb[edge.vertices[0]], x_rb[edge.vertices[1]]));
				this->friction.rb_d_edge_point.contact.T.push_back(projection_matrix_point_edge(x_cloth[p.idx], x_rb[edge.vertices[0]], x_rb[edge.vertices[1]]));
				this->friction.rb_d_edge_point.contact.mu.push_back(mu);
				this->friction.rb_d_edge_point.contact.fn.push_back(mollifier_f(edge, ep.edge)*force(pair.distance));
			}
		}
	}
	//// Edge - Edge
	for (const auto& pair : proximity.edge_edge.edge_edge) {
		const tmcd::Edge& ea = pair.first;
		const tmcd::Edge& eb = pair.second;

		const tmcd::Edge& edge_rb = (ea.set == this->rigid_bodies_id) ? ea : eb;
		const tmcd::Edge& edge_cloth = (ea.set == this->rigid_bodies_id) ? eb : ea;

		const int rb_idx = this->rigid_bodies->mesh.get_mesh_containing_vertex(edge_rb.vertices[0]);
		const double mu = 0.5 * (mu_rb[rb_idx] + mu_cloth[edge_cloth.vertices[0]]);
		if (mu > 0.0 && are_not_almost_parallel(edge_rb, edge_cloth)) {
			this->friction.rb_d_edge_edge.conn.numbered_push_back({ rb_idx, edge_rb.vertices[0], edge_rb.vertices[1], edge_cloth.vertices[0], edge_cloth.vertices[1] });
			this->friction.rb_d_edge_edge.bary.push_back(barycentric_edge_edge(x_rb[edge_rb.vertices[0]], x_rb[edge_rb.vertices[1]], x_cloth[edge_cloth.vertices[0]], x_cloth[edge_cloth.vertices[1]]));
			this->friction.rb_d_edge_edge.contact.T.push_back(projection_matrix_edge_edge(x_rb[edge_rb.vertices[0]], x_rb[edge_rb.vertices[1]], x_cloth[edge_cloth.vertices[0]], x_cloth[edge_cloth.vertices[1]]));
			this->friction.rb_d_edge_edge.contact.mu.push_back(mu);
			this->friction.rb_d_edge_edge.contact.fn.push_back(mollifier_f(edge_rb, edge_cloth)*force(pair.distance));
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
	if (!this->cloth->is_empty()) {
		this->cloth_id = this->id.add_mesh(&this->cloth->collision_x1[0][0], (int)this->cloth->collision_x1.size(), &this->cloth->model.mesh.connectivity[0][0], this->cloth->model.mesh.get_n_elements(), &this->cloth->edges.connectivity[0][0], (int)this->cloth->edges.get_n_edges());
		this->id.add_blacklist(this->cloth_id, this->cloth_id);
	}
	if (!this->rigid_bodies->is_empty()) {
		this->rigid_bodies_id = this->id.add_mesh(&this->rigid_bodies->collision_x1[0][0], (int)this->rigid_bodies->collision_x1.size(), &this->rigid_bodies->mesh.connectivity[0][0], this->rigid_bodies->mesh.get_n_elements(), &this->rigid_bodies->edges.connectivity[0][0], (int)this->rigid_bodies->edges.get_n_edges());
		this->id.add_blacklist(this->rigid_bodies_id, this->rigid_bodies_id);
	}
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
	auto get_rb_X = [&](const std::vector<symx::Index>& indices, symx::Energy& energy)
	{
		return energy.make_vectors(this->rigid_bodies->mesh.vertices, indices);
	};
	auto get_d_X = [&](const std::vector<symx::Index>& indices, symx::Energy& energy)
	{
		return energy.make_vectors(this->cloth->model.mesh.vertices, indices);
	};
	auto get_rb_edge_point = [&](const std::vector<symx::Index>& conn, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		// conn = { conn["a_e0"], conn["a_e1"], conn["a_p"] }
		std::vector<symx::Vector> A = get_rb_x1(conn, rb_idx, dt, energy);
		std::vector<symx::Vector> EA = { A[0], A[1] };
		symx::Vector P = A[2];
		std::vector<symx::Vector> EA_REST = get_rb_X({ conn[0], conn[1] }, energy);
		return std::make_tuple(EA_REST, EA, P);
	};
	auto get_rb_edge = [&](const std::vector<symx::Index>& conn, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		// conn = { conn["a_e0"], conn["a_e1"] }
		std::vector<symx::Vector> EA = get_rb_x1(conn, rb_idx, dt, energy);
		std::vector<symx::Vector> EA_REST = get_rb_X(conn, energy);
		return std::make_tuple(EA_REST, EA);
	};
	auto get_d_edge_point = [&](const std::vector<symx::Index>& conn, const symx::Scalar& dt, symx::Energy& energy)
	{
		std::vector<symx::Vector> A = get_d_x1(conn, dt, energy);
		std::vector<symx::Vector> EA = { A[0], A[1] };
		symx::Vector P = A[2];
		std::vector<symx::Vector> EA_REST = get_d_X({ conn[0], conn[1] }, energy);
		return std::make_tuple(EA_REST, EA, P);
	};
	auto get_d_edge = [&](const std::vector<symx::Index>& conn, const symx::Scalar& dt, symx::Energy& energy)
	{
		std::vector<symx::Vector> EA = get_d_x1({ conn[0], conn[1] }, dt, energy);
		std::vector<symx::Vector> EA_REST = get_d_X({ conn[0], conn[1] }, energy);
		return std::make_tuple(EA_REST, EA);
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
		symx::Scalar E = edge_edge_mollifier(EA, EB, EA_REST, EB_REST) * barrier_energy(d, dhat, k);
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

	// Edge - Edge
	//// Point - Point
	sim.global_energy.add_energy("collision_rb_d_ee_point_point", this->contacts.edge_edge.rb_d_point_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			auto [EA_REST, EA, P] = get_rb_edge_point({ conn["rb_e0"], conn["rb_e1"], conn["rb_p"] }, conn["rb"], dt, energy);
			auto [EB_REST, EB, Q] = get_d_edge_point({ conn["e0"], conn["e1"], conn["q"] }, dt, energy);
			symx::Scalar d = distance_point_point(P, Q);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);

	//// Point - Edge
	sim.global_energy.add_energy("collision_rb_d_ee_point_edge", this->contacts.edge_edge.rb_d_point_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			auto [EA_REST, EA, P] = get_rb_edge_point({ conn["rb_e0"], conn["rb_e1"], conn["rb_p"] }, conn["rb"], dt, energy);
			auto [EB_REST, EB] = get_d_edge({ conn["e0"], conn["e1"] }, dt, energy);
			symx::Scalar d = distance_point_line(P, EB[0], EB[1]);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);

	//// Edge - Edge
	sim.global_energy.add_energy("collision_rb_d_ee_edge_edge", this->contacts.edge_edge.rb_d_edge_edge,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			auto [EA_REST, EA] = get_rb_edge({ conn["rb_e0"], conn["rb_e1"] }, conn["rb"], dt, energy);
			auto [EB_REST, EB] = get_d_edge({ conn["e0"], conn["e1"] }, dt, energy);
			symx::Scalar d = distance_line_line(EA[0], EA[1], EB[0], EB[1]);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);

	//// D -> RB: Point - Edge
	sim.global_energy.add_energy("collision_rb_d_ee_edge_point", this->contacts.edge_edge.rb_d_edge_point,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			auto [EB_REST, EB, Q] = get_d_edge_point({ conn["e0"], conn["e1"], conn["q"] }, dt, energy);
			auto [EA_REST, EA] = get_rb_edge({ conn["rb_e0"], conn["rb_e1"] }, conn["rb"], dt, energy);
			symx::Scalar d = distance_point_line(Q, EA[0], EA[1]);
			set_edge_dge_mollified_barrier_energy(EA, EB, EA_REST, EB_REST, d, energy);
		}
	);
}
void stark::models::Interactions::_energies_friction(Stark& sim)
{
	// Common symbol creation and manipulation functions --------------------------------------------------------------------------------------------------
	auto get_rb_v1 = [&](const std::vector<symx::Index>& indices, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Vector v1 = energy.make_dof_vector(this->rigid_bodies->dof_v, this->rigid_bodies->v1, rb_idx);
		symx::Vector w1 = energy.make_dof_vector(this->rigid_bodies->dof_w, this->rigid_bodies->w1, rb_idx);
		symx::Vector t0 = energy.make_vector(this->rigid_bodies->t0, rb_idx);
		symx::Vector q0 = energy.make_vector(this->rigid_bodies->q0_, rb_idx);

		symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
		symx::Vector t1 = time_integration(t0, v1, dt);

		std::vector<symx::Vector> x_loc = energy.make_vectors(this->rigid_bodies->mesh.vertices, indices);

		std::vector<symx::Vector> v1_glob;
		for (const symx::Vector& x_loc_a : x_loc) {
			symx::Vector x1a = local_to_global_point(x_loc_a, t1, R1);
			symx::Vector r1a = x1a - t1;
			symx::Vector v1a = global_point_velocity_in_rigib_body(v1, w1, r1a);
			v1_glob.push_back(v1a);
		}
		return v1_glob;
	};
	auto get_d_v1 = [&](const std::vector<symx::Index>& conn, symx::Energy& energy)
	{
		return energy.make_dof_vectors(this->cloth->dof, this->cloth->model.v1, conn);
	};
	auto set_friction_energy = [&](const symx::Vector& v, const symx::Index& contact_idx, const RB_Deformable_Friction::Contact& contact, const symx::Scalar& dt, symx::Energy& energy)
	{
		symx::Matrix T = energy.make_matrix(contact.T, { 2, 3 }, contact_idx);
		symx::Scalar mu = energy.make_scalar(contact.mu, contact_idx);
		symx::Scalar fn = energy.make_scalar(contact.fn, contact_idx);
		symx::Scalar epsv = energy.make_scalar(sim.settings.contact.friction_stick_slide_threshold);
		symx::Scalar perturbation = energy.make_scalar(sim.settings.contact.friction_displacement_perturbation);

		symx::Vector vt = T * v;
		symx::Vector ut = vt * dt;
		ut[0] += 1.13 * perturbation;
		ut[1] -= 1.07 * perturbation;
		symx::Scalar u = ut.norm();

		symx::Scalar epsu = dt * epsv;
		if (sim.settings.contact.better_friction_mode) {
			symx::Scalar k = mu * fn / epsu;
			symx::Scalar eps = mu * fn / (2.0 * k);

			symx::Scalar E_stick = 0.5 * k * u.powN(2);
			symx::Scalar E_slide = mu * fn * (u - eps);
			symx::Scalar E = symx::branch(u < epsu, E_stick, E_slide);
			energy.set(E);
		}
		else {
			symx::Scalar E_stick = mu * fn * (-u * u * u / (3.0 * epsu.powN(2)) + u * u / epsu + epsu / 3.0);
			symx::Scalar E_slide = mu * fn * u;
			symx::Scalar E = symx::branch(u < epsu, E_stick, E_slide);
			energy.set(E);
		}

		energy.activate(sim.settings.contact.collisions_enabled && sim.settings.contact.friction_enabled);
	};
	// ----------------------------------------------------------------------------------------------------------------------------------------------------


	// Point - Point
	sim.global_energy.add_energy("friction_rb_d_point_point", this->friction.rb_d_point_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> VP = get_rb_v1({ conn["rb_p"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> VQ = get_d_v1({ conn["q"] }, energy);

			symx::Vector v = VQ[0] - VP[0];
			set_friction_energy(v, conn["idx"], this->friction.rb_d_point_point.contact, dt, energy);
		}
	);
	// Point - Edge
	sim.global_energy.add_energy("friction_rb_d_point_edge", this->friction.rb_d_point_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> VP = get_rb_v1({ conn["p"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> VE = get_d_v1({ conn["e0"], conn["e1"] }, energy);
			symx::Vector bary = energy.make_vector(this->friction.rb_d_point_edge.bary, conn["idx"]);

			symx::Vector vp = VP[0];
			symx::Vector vq = bary[0] * VE[0] + bary[1] * VE[1];
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn["idx"], this->friction.rb_d_point_edge.contact, dt, energy);
		}
	);
	// Point - Triangle
	sim.global_energy.add_energy("friction_rb_d_point_triangle", this->friction.rb_d_point_triangle.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> VP = get_rb_v1({ conn["p"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> VT = get_d_v1({ conn["t0"], conn["t1"], conn["t2"] }, energy);
			symx::Vector bary = energy.make_vector(this->friction.rb_d_point_triangle.bary, conn["idx"]);

			symx::Vector vp = VP[0];
			symx::Vector vq = bary[0] * VT[0] + bary[1] * VT[1] + bary[2] * VT[2];
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn["idx"], this->friction.rb_d_point_triangle.contact, dt, energy);
		}
	);
	// Edge - Edge
	sim.global_energy.add_energy("friction_rb_d_edge_edge", this->friction.rb_d_edge_edge.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> VEA = get_rb_v1({ conn["rb_e0"], conn["rb_e1"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> VEB = get_d_v1({ conn["e0"], conn["e1"] }, energy);
			symx::Vector bary = energy.make_vector(this->friction.rb_d_edge_edge.bary, conn["idx"]);

			symx::Vector vp = VEA[0] + bary[0] * (VEA[1] - VEA[0]);
			symx::Vector vq = VEB[0] + bary[1] * (VEB[1] - VEB[0]);
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn["idx"], this->friction.rb_d_edge_edge.contact, dt, energy);
		}
	);
	// D -> RB: Point - Edge
	sim.global_energy.add_energy("friction_rb_d_edge_point", this->friction.rb_d_edge_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> VE = get_rb_v1({ conn["e0"], conn["e1"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> VP = get_d_v1({ conn["p"] }, energy);
			symx::Vector bary = energy.make_vector(this->friction.rb_d_edge_point.bary, conn["idx"]);

			symx::Vector vp = VP[0];
			symx::Vector vq = bary[0] * VE[0] + bary[1] * VE[1];
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn["idx"], this->friction.rb_d_edge_point.contact, dt, energy);
		}
	);
	// D -> RB: Point - Triangle
	sim.global_energy.add_energy("friction_rb_d_triangle_point", this->friction.rb_d_triangle_point.conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			symx::Scalar dt = energy.make_scalar(sim.settings.simulation.adaptive_time_step.value);
			std::vector<symx::Vector> VT = get_rb_v1({ conn["t0"], conn["t1"], conn["t2"] }, conn["rb"], dt, energy);
			std::vector<symx::Vector> VP = get_d_v1({ conn["p"] }, energy);
			symx::Vector bary = energy.make_vector(this->friction.rb_d_triangle_point.bary, conn["idx"]);

			symx::Vector vp = VP[0];
			symx::Vector vq = bary[0] * VT[0] + bary[1] * VT[1] + bary[2] * VT[2];
			symx::Vector v = vq - vp;
			set_friction_energy(v, conn["idx"], this->friction.rb_d_triangle_point.contact, dt, energy);
		}
	);
}
