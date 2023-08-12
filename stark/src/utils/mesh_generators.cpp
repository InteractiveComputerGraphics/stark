#include "mesh_generators.h"
#include <limits>

#include <par_shapes/par_shapes.h>

#include "mesh_utils.h"


stark::utils::Mesh as_mesh(par_shapes_mesh* pm)
{
	stark::utils::Mesh mesh;

	mesh.vertices.resize(pm->npoints);
	for (int i = 0; i < pm->npoints; i++) {
		mesh.vertices[i] = { pm->points[3*i], pm->points[3*i + 1], pm->points[3*i + 2] };
	}

	mesh.triangles.resize(pm->ntriangles);
	for (int i = 0; i < pm->ntriangles; i++) {
		mesh.triangles[i] = { pm->triangles[3*i], pm->triangles[3*i + 1], pm->triangles[3*i + 2] };
	}

	par_shapes_free_mesh(pm);
	return mesh;
}


stark::utils::Mesh stark::utils::make_sphere(const double radius, const int subdivisions)
{
	Mesh m = as_mesh(par_shapes_create_subdivided_sphere(subdivisions));
	scale(m.vertices, radius);
	return m;
}
stark::utils::Mesh stark::utils::make_box(const Eigen::Vector3d& size)
{
	Mesh m = as_mesh(par_shapes_create_cube());
	move(m.vertices, {-0.5, -0.5, -0.5});
	scale(m.vertices, size);
	return m;
}
stark::utils::Mesh stark::utils::make_box(const double size)
{
	return make_box({size, size, size});
}
stark::utils::Mesh stark::utils::make_cylinder(const double radius, const double full_height, const int slices, const int stacks)
{
	const std::array<float, 3> c_t = {0, 0, 1};
	const std::array<float, 3> c_b = {0, 0, 0};
	const std::array<float, 3> n = {0, 0, 1};

	par_shapes_mesh* pc = par_shapes_create_cylinder(slices, stacks);
	par_shapes_mesh* pd_top = par_shapes_create_disk(1, slices, c_t.data(), n.data());
	par_shapes_mesh* pd_bottom = par_shapes_create_disk(1, slices, c_b.data(), n.data());
	par_shapes_merge_and_free(pc, pd_top);
	par_shapes_merge_and_free(pc, pd_bottom);
	pc = par_shapes_weld(pc, 10.0f*std::numeric_limits<float>::epsilon(), nullptr);

	Mesh m = as_mesh(pc);
	move(m.vertices, {0.0, 0.0, -0.5});
	scale(m.vertices, {radius, radius, full_height });
	return m;
}
stark::utils::Mesh stark::utils::make_torus(const double outer_radius, const double inner_radius, const int slices, const int stacks)
{
	Mesh m = as_mesh(par_shapes_create_torus(inner_radius/outer_radius, slices, stacks));
	scale(m.vertices, outer_radius);
	return m;
}
stark::utils::Mesh stark::utils::make_knot(const double scale_, const double inner_radius, const int slices, const int stacks)
{
	Mesh m = as_mesh(par_shapes_create_trefoil_knot(inner_radius/scale_, slices, stacks));
	scale(m.vertices, scale_);
	return m;
}
