#include "mesh_generators.h"

#include <par_shapes/par_shapes.h>


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


stark::utils::Mesh stark::utils::make_sphere(const int subdivisions)
{
	return as_mesh(par_shapes_create_subdivided_sphere(subdivisions));
}