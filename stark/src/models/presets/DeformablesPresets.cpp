#include "DeformablesPresets.h"

#include "../../utils/include.h"


stark::DeformablesPresets::DeformablesPresets(core::Stark& stark, std::shared_ptr<Deformables> deformables, std::shared_ptr<Interactions> interactions)
	: deformables(deformables), interactions(interactions)
{
}

stark::Line::Handler stark::DeformablesPresets::add_line(const std::string& output_label, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 2>>& segments, const Line::Params& params)
{
	PointSetHandler point_set = this->deformables->point_sets->add(vertices);
	EnergyLumpedInertia::Handler inertia = this->deformables->lumped_inertia->add(point_set, segments, params.inertia);
	EnergySegmentStrain::Handler strain = this->deformables->segment_strain->add(point_set, segments, params.strain);
	ContactHandler contact = this->interactions->contact->add_edges(point_set, segments, params.contact);

	if (output_label != "") {
		this->deformables->output->add_segment_mesh(output_label, point_set, segments);
	}

	return { point_set, inertia, strain, contact };
}
stark::Line::VCH stark::DeformablesPresets::add_line_as_segments(const std::string& output_label, const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments, const Line::Params& params)
{
	auto [vertices, segments] = generate_segment_line(begin, end, n_segments);
	Line::Handler handler = this->add_line(output_label, vertices, segments, params);
	return { vertices, segments, handler };
}

stark::Surface::Handler stark::DeformablesPresets::add_surface(const std::string& output_label, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Surface::Params& params)
{
	PointSetHandler point_set = this->deformables->point_sets->add(vertices);
	EnergyLumpedInertia::Handler inertia = this->deformables->lumped_inertia->add(point_set, triangles, params.inertia);
	EnergyTriangleStrain::Handler strain = this->deformables->triangle_strain->add(point_set, triangles, params.strain);
	EnergyDiscreteShells::Handler bending = this->deformables->discrete_shells->add(point_set, triangles, params.bending);
	ContactHandler contact = this->interactions->contact->add_triangles(point_set, triangles, params.contact);

	if (output_label != "") {
		this->deformables->output->add_triangle_mesh(output_label, point_set, triangles);
	}

	return { point_set, inertia, strain, bending, contact };
}
stark::Surface::VCH stark::DeformablesPresets::add_surface_grid(const std::string& output_label, const Eigen::Vector2d& dim, const std::array<int, 2>& subdivisions, const Surface::Params& params)
{
	auto [vertices, triangles] = generate_triangle_grid({ 0.0, 0.0 }, dim, subdivisions);
	Surface::Handler handler = this->add_surface(output_label, vertices, triangles, params);
	return { vertices, triangles, handler };
}
stark::PrescribedSurface::Handler stark::DeformablesPresets::add_prescribed_surface(const std::string& output_label, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const PrescribedSurface::Params& params)
{
	PointSetHandler point_set = this->deformables->point_sets->add(vertices);
	EnergyPrescribedPositions::Handler prescribed = this->deformables->prescribed_positions->add(point_set, point_set.all(), params.prescribed);
	ContactHandler contact = this->interactions->contact->add_triangles(point_set, triangles, params.contact);
	contact.disable_collision(contact);  // Disable self-collisions

	if (output_label != "") {
		this->deformables->output->add_triangle_mesh(output_label, point_set, triangles);
	}

	return { point_set, prescribed, contact };
}

stark::Volume::Handler stark::DeformablesPresets::add_volume(const std::string& output_label, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets, const Volume::Params& params)
{
	auto [surface_triangles, tri_to_tet_map] = stark::find_surface(vertices, tets);

	PointSetHandler point_set = this->deformables->point_sets->add(vertices);
	EnergyLumpedInertia::Handler inertia = this->deformables->lumped_inertia->add(point_set, tets, params.inertia);
	EnergyTetStrain::Handler strain = this->deformables->tet_strain->add(point_set, tets, params.strain);
	ContactHandler contact = this->interactions->contact->add_triangles(point_set, surface_triangles, tri_to_tet_map, params.contact);

	if (output_label != "") {
		this->deformables->output->add_triangle_mesh(output_label, point_set, surface_triangles, tri_to_tet_map);
	}

	return { point_set, inertia, strain, contact };
}
stark::Volume::VCH stark::DeformablesPresets::add_volume_grid(const std::string& output_label, const Eigen::Vector3d& dim, const std::array<int, 3>& subdivisions, const Volume::Params& params)
{
	auto [vertices, tets] = generate_tet_grid({ 0.0, 0.0, 0.0 }, dim, subdivisions);
	Volume::Handler handler = this->add_volume(output_label, vertices, tets, params);
	return { vertices, tets, handler };
}
