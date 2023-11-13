#include "EnergyFrictionalContact.h"

#include "../time_integration.h"
#include "../rigidbody_transformations.h"
#include "../distances.h"
#include "../../utils/mesh_utils.h"


void stark::models::EnergyFrictionalContact::declare(Stark& stark)
{
	// Callbacks
	stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step__update_friction_contacts(stark); });
	stark.callbacks.before_energy_evaluation.push_back([&]() { this->_before_energy_evaluation__update_contacts(stark); });
	stark.callbacks.is_state_valid.push_back([&]() { return this->_is_valid_configuration(stark); });

	// Energy declarations
	this->_energies_contact_deformables(stark);
	this->_energies_contact_rb(stark);
	this->_energies_contact_rb_deformables(stark);

	this->_energies_friction_deformables(stark);
	this->_energies_friction_rb(stark);
	this->_energies_friction_rb_deformables(stark);
}

void stark::models::EnergyFrictionalContact::add_points(Id& id)
{
	// Points
	const int n_points = this->dyn->size(id);
	std::vector<std::array<int, 1>> points(n_points);
	for (int i = 0; i < n_points; i++) {
		points[i] = { i };
	}

	// Add
	const int offset = this->dyn->get_begin(id);
	const int local_idx = this->points.append(points, offset);
	this->edges.append_empty();
	this->triangles.append_empty();

	// Set local index
	id.set_local_idx("EnergyFrictionalContact", local_idx);
}

void stark::models::EnergyFrictionalContact::add_edges_and_points(Id& id, const std::vector<std::array<int, 2>>& edges)
{
	// Points
	const int n_points = this->dyn->size(id);
	std::vector<std::array<int, 1>> points(n_points);
	for (int i = 0; i < n_points; i++) {
		points[i] = { i };
	}

	// Add
	const int offset = this->dyn->get_begin(id);
	const int local_idx = this->points.append(points, offset);
	this->edges.append(edges, offset);
	this->triangles.append_empty();

	// Set local index
	id.set_local_idx("EnergyFrictionalContact", local_idx);
}

void stark::models::EnergyFrictionalContact::add_triangles_edges_and_points(Id& id, const std::vector<std::array<int, 3>>& triangles)
{
	// Points
	const int n_points = this->dyn->size(id);
	std::vector<std::array<int, 1>> points(n_points);
	for (int i = 0; i < n_points; i++) {
		points[i] = { i };
	}

	// Edges
	std::vector<std::array<int, 2>> edges;
	utils::find_edges_from_simplices(edges, triangles, n_points);
	
	// Add
	const int offset = this->dyn->get_begin(id);
	const int local_idx = this->points.append(points, offset);
	this->edges.append(edges, offset);
	this->triangles.append(triangles, offset);

	// Set local index
	id.set_local_idx("EnergyFrictionalContact", local_idx);
}



/* ========================================================================================== */
/* ===================================  SYMX DEFINITIONS  =================================== */
/* ========================================================================================== */
void stark::models::EnergyFrictionalContact::_energies_contact_deformables(Stark& stark)
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
void stark::models::EnergyFrictionalContact::_energies_contact_rb(Stark& stark)
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
			auto [ea, ea_rest] = this->_get_rb_edge(energy, stark, conn["a"], { conn["ea0"], conn["ea1"] });
			auto [eb, eb_rest] = this->_get_rb_edge(energy, stark, conn["b"], { conn["eb0"], conn["eb1"] });
			symx::Scalar d = distance_line_line(ea[0], ea[1], eb[0], eb[1]);
			this->_set_edge_edge_mollified_barrier_potential(energy, stark, d, ea, eb, ea_rest, eb_rest);
		}
	);
}
void stark::models::EnergyFrictionalContact::_energies_contact_rb_deformables(Stark& stark)
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

void stark::models::EnergyFrictionalContact::_energies_friction_deformables(Stark& stark)
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
void stark::models::EnergyFrictionalContact::_energies_friction_rb(Stark& stark)
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
void stark::models::EnergyFrictionalContact::_energies_friction_rb_deformables(Stark& stark)
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
symx::Scalar stark::models::EnergyFrictionalContact::_barrier_potential(const symx::Scalar& d, const symx::Scalar& dhat, const symx::Scalar& k)
{
	if (this->ipc_barrier_type == IPCBarrierType::Cubic) {
		return k * (dhat - d).powN(3);
	}
	else if (this->ipc_barrier_type == IPCBarrierType::Log) {
		return -k * (dhat - d).powN(2) * log(d / dhat);
	}
}
symx::Scalar stark::models::EnergyFrictionalContact::_edge_edge_mollifier(const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest)
{
	symx::Scalar eps_x = 1e-3 * (ea_rest[0] - ea_rest[1]).squared_norm() * (eb_rest[0] - eb_rest[1]).squared_norm();
	symx::Scalar x = (ea[1] - ea[0]).cross3(eb[1] - eb[0]).squared_norm();
	symx::Scalar x_div_eps_x = x / eps_x;
	symx::Scalar f = (-x_div_eps_x + 2.0) * x_div_eps_x;
	symx::Scalar mollifier = symx::branch(x > eps_x, 1.0, f);
	return mollifier;
}
symx::Scalar stark::models::EnergyFrictionalContact::_friction_potential(const symx::Vector& v, const symx::Scalar& fn, const symx::Scalar& mu, const symx::Matrix& T, const symx::Scalar& epsv, const symx::Scalar& dt)
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
}


/* ====================================================================================== */
/* ===================================  SYMX SETTERS  =================================== */
/* ====================================================================================== */
void stark::models::EnergyFrictionalContact::_set_barrier_potential(symx::Energy& energy, const Stark& stark, const symx::Scalar& d)
{
	symx::Scalar k = energy.make_scalar(stark.settings.contact.adaptive_contact_stiffness.value);
	symx::Scalar dhat = energy.make_scalar(stark.settings.contact.dhat);
	symx::Scalar E = this->_barrier_potential(d, dhat, k);
	energy.set(E);
	energy.activate(stark.settings.contact.collisions_enabled);
}
void stark::models::EnergyFrictionalContact::_set_edge_edge_mollified_barrier_potential(symx::Energy& energy, const Stark& stark, const symx::Scalar& d, const std::vector<symx::Vector>& ea, const std::vector<symx::Vector>& eb, const std::vector<symx::Vector>& ea_rest, const std::vector<symx::Vector>& eb_rest)
{
	symx::Scalar k = energy.make_scalar(stark.settings.contact.adaptive_contact_stiffness.value);
	symx::Scalar dhat = energy.make_scalar(stark.settings.contact.dhat);
	symx::Scalar E = this->_edge_edge_mollifier(ea, eb, ea_rest, eb_rest) * this->_barrier_potential(d, dhat, k);
	energy.set(E);
	energy.activate(stark.settings.contact.collisions_enabled);
}
void stark::models::EnergyFrictionalContact::_set_friction_potential(symx::Energy& energy, const Stark& stark, const symx::Vector& v, const symx::Index& contact_idx, const FrictionContact& contact)
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
void stark::models::EnergyFrictionalContact::_set_friction_point_edge(symx::Energy& energy, const Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& ve, const symx::Index& contact_idx, FrictionPointEdge& data)
{
	symx::Vector bary = energy.make_vector(data.bary, contact_idx);

	symx::Vector va = vp;
	symx::Vector vb = bary[0] * ve[0] + bary[1] * ve[1];
	symx::Vector v = vb - va;
	this->_set_friction_potential(energy, stark, v, contact_idx, data.contact);
}
void stark::models::EnergyFrictionalContact::_set_friction_point_triangle(symx::Energy& energy, const Stark& stark, const symx::Vector& vp, const std::vector<symx::Vector>& vt, const symx::Index& contact_idx, FrictionPointTriangle& data)
{
	symx::Vector bary = energy.make_vector(data.bary, contact_idx);

	symx::Vector va = vp;
	symx::Vector vb = bary[0] * vt[0] + bary[1] * vt[1] + bary[2] * vt[2];
	symx::Vector v = vb - va;
	this->_set_friction_potential(energy, stark, v, contact_idx, data.contact);
}
void stark::models::EnergyFrictionalContact::_set_friction_edge_edge(symx::Energy& energy, const Stark& stark, const std::vector<symx::Vector>& vea, const std::vector<symx::Vector>& veb, const symx::Index& contact_idx, FrictionEdgeEdge& data)
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
std::vector<symx::Vector> stark::models::EnergyFrictionalContact::_get_rb_v1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	symx::Vector v1 = energy.make_dof_vector(this->rb->dof_v, this->rb->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->rb->dof_w, this->rb->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->rb->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->rb->q0_, rb_idx);

	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
	symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
	symx::Vector t1 = time_integration(t0, v1, dt);

	std::vector<symx::Vector> x_loc = energy.make_vectors(this->rb->collision_mesh.vertices, conn);

	std::vector<symx::Vector> v1_glob;
	for (const symx::Vector& x_loc_a : x_loc) {
		symx::Vector x1a = local_to_global_point(x_loc_a, t1, R1);
		symx::Vector r1a = x1a - t1;
		symx::Vector v1a = global_point_velocity_in_rigib_body(v1, w1, r1a);
		v1_glob.push_back(v1a);
	}
	return v1_glob;
}
std::vector<symx::Vector> stark::models::EnergyFrictionalContact::_get_rb_x1(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	symx::Vector v1 = energy.make_dof_vector(this->rb->dof_v, this->rb->v1, rb_idx);
	symx::Vector w1 = energy.make_dof_vector(this->rb->dof_w, this->rb->w1, rb_idx);
	symx::Vector t0 = energy.make_vector(this->rb->t0, rb_idx);
	symx::Vector q0 = energy.make_vector(this->rb->q0_, rb_idx);

	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
	symx::Matrix R1 = quat_time_integration_as_rotation_matrix(q0, w1, dt);
	symx::Vector t1 = time_integration(t0, v1, dt);

	std::vector<symx::Vector> x_loc = energy.make_vectors(this->rb->collision_mesh.vertices, conn);

	std::vector<symx::Vector> x1;
	for (const symx::Vector& x_loc_a : x_loc) {
		x1.push_back(local_to_global_point(x_loc_a, t1, R1));
	}
	return x1;
}
std::vector<symx::Vector> stark::models::EnergyFrictionalContact::_get_rb_X(symx::Energy& energy, const std::vector<symx::Index>& conn)
{
	return energy.make_vectors(this->rb->collision_mesh.vertices, conn);
}
std::array<std::vector<symx::Vector>, 2> stark::models::EnergyFrictionalContact::_get_rb_edge(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"] }
	std::vector<symx::Vector> ea = this->_get_rb_x1(energy, stark, rb_idx, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_rb_X(energy, { conn[0], conn[1] });
	return { ea, ea_rest };
}
std::array<std::vector<symx::Vector>, 3> stark::models::EnergyFrictionalContact::_get_rb_edge_point(symx::Energy& energy, const Stark& stark, const symx::Index& rb_idx, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"], conn["p"] }
	std::vector<symx::Vector> ea = this->_get_rb_x1(energy, stark, rb_idx, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_rb_X(energy, { conn[0], conn[1] });
	std::vector<symx::Vector> p = this->_get_rb_x1(energy, stark, rb_idx, { conn[2] });
	return { ea, ea_rest, p };
}
std::vector<symx::Vector> stark::models::EnergyFrictionalContact::_get_d_v1(symx::Energy& energy, const std::vector<symx::Index>& conn)
{
	return energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, conn);
}
std::vector<symx::Vector> stark::models::EnergyFrictionalContact::_get_d_x1(symx::Energy& energy, const Stark& stark, const std::vector<symx::Index>& conn)
{
	std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, conn);
	std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, conn);
	symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
	return time_integration(x0, v1, dt);
}
std::vector<symx::Vector> stark::models::EnergyFrictionalContact::_get_d_X(symx::Energy& energy, const std::vector<symx::Index>& conn)
{
	return energy.make_vectors(this->dyn->X.data, conn);
}
std::array<std::vector<symx::Vector>, 2> stark::models::EnergyFrictionalContact::_get_d_edge(symx::Energy& energy, const Stark& stark, const std::vector<symx::Index>& conn)
{
	// conn = { conn["ea0"], conn["ea1"] }
	std::vector<symx::Vector> ea = this->_get_d_x1(energy, stark, { conn[0], conn[1] });
	std::vector<symx::Vector> ea_rest = this->_get_d_X(energy, { conn[0], conn[1] });
	return { ea, ea_rest };
}
std::array<std::vector<symx::Vector>, 3> stark::models::EnergyFrictionalContact::_get_d_edge_point(symx::Energy& energy, const Stark& stark, const std::vector<symx::Index>& conn)
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
std::string stark::models::EnergyFrictionalContact::_get_contact_label(const std::string physical_system, const std::string pair) const
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
std::string stark::models::EnergyFrictionalContact::_get_friction_label(const std::string physical_system, const std::string pair) const
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

