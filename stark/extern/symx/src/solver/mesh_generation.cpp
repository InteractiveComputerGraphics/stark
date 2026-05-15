#include "mesh_generation.h"
#include "mesh_ordering.h"

using namespace symx;

template <typename FLOAT>
void reorder_RCM(FEMMesh<FLOAT>& mesh)
{
    if (mesh.connectivity_stride <= 0) {
        throw std::runtime_error("reorder_RCM: mesh.connectivity_stride must be set to a positive value.");
    }
    const int num_nodes = static_cast<int>(mesh.vertices.size());
    const int n_elements = static_cast<int>(mesh.connectivity.size() / mesh.connectivity_stride);
    auto res = reorder_connectivity_RCM(mesh.connectivity.data(), mesh.connectivity_stride, n_elements, num_nodes);
    apply_permutation_inplace(mesh.vertices, res.old_to_new);
}
template <typename FLOAT>
void reorder_RCM(TriangleMesh<FLOAT>& mesh)
{
    auto res = reorder_connectivity_RCM<3>(mesh.triangles, (int)mesh.vertices.size());
    apply_permutation_inplace(mesh.vertices, res.old_to_new);
}

std::vector<int> symx::enumerate(const std::vector<int> &connectivity, int stride)
{
    const int n_elements = static_cast<int>(connectivity.size() / stride);
    const int enumerated_stride = stride + 1;
    std::vector<int> out;
    out.reserve(n_elements * enumerated_stride);
    for (int i = 0; i < n_elements; ++i) {
        out.push_back(i);
        out.insert(out.end(), connectivity.begin() + i * stride, connectivity.begin() + (i + 1) * stride);
    }
    return out;
}


template <typename FLOAT>
FEMMesh<FLOAT> symx::generate_cuboid_Tet4_mesh(
    const Eigen::Matrix<FLOAT, 3, 1> &size,
    const std::array<int32_t, 3> &elements_per_axis)
{
    using Vector3S = Eigen::Matrix<FLOAT, 3, 1>;
    FEMMesh<FLOAT> mesh;
    mesh.element_type = FEM_Element::Tet4;
    mesh.connectivity_stride = 4;
    Vector3S min_corner = -0.5 * size;
    Vector3S epa_cast(static_cast<FLOAT>(elements_per_axis[0]), static_cast<FLOAT>(elements_per_axis[1]), static_cast<FLOAT>(elements_per_axis[2]));
    Vector3S delta = size.cwiseQuotient(epa_cast);

    for (int i = 0; i <= elements_per_axis[0]; ++i) {
        for (int j = 0; j <= elements_per_axis[1]; ++j) {
            for (int k = 0; k <= elements_per_axis[2]; ++k) {
                mesh.vertices.emplace_back(min_corner + Vector3S((FLOAT)i, (FLOAT)j, (FLOAT)k).cwiseProduct(delta));
            }
        }
    }

    auto index = [&](int i, int j, int k) -> int {
        return i * (elements_per_axis[1] + 1) * (elements_per_axis[2] + 1) +
            j * (elements_per_axis[2] + 1) + k;
        };

    for (int i = 0; i < elements_per_axis[0]; ++i) {
        for (int j = 0; j < elements_per_axis[1]; ++j) {
            for (int k = 0; k < elements_per_axis[2]; ++k) {
                int v0 = index(i, j, k);
                int v1 = index(i + 1, j, k);
                int v2 = index(i + 1, j + 1, k);
                int v3 = index(i, j + 1, k);
                int v4 = index(i, j, k + 1);
                int v5 = index(i + 1, j, k + 1);
                int v6 = index(i + 1, j + 1, k + 1);
                int v7 = index(i, j + 1, k + 1);

                // Split the hexahedron into 5 tetrahedra
                // (Paper: How to Subdivide Pyramids, Prisms and Hexahedra into Tetrahedra)
                mesh.connectivity.insert(mesh.connectivity.end(), { v0, v1, v6, v5 });
                mesh.connectivity.insert(mesh.connectivity.end(), { v0, v1, v2, v6 });
                mesh.connectivity.insert(mesh.connectivity.end(), { v0, v2, v3, v6 });
                mesh.connectivity.insert(mesh.connectivity.end(), { v0, v5, v6, v4 });
                mesh.connectivity.insert(mesh.connectivity.end(), { v0, v6, v3, v7 });
                mesh.connectivity.insert(mesh.connectivity.end(), { v0, v6, v7, v4 });
            }
        }
    }
    mesh.n_elements = static_cast<int>(mesh.connectivity.size() / mesh.connectivity_stride);

    reorder_RCM(mesh);
    return mesh;
}
template <typename FLOAT>
FEMMesh<FLOAT> symx::generate_cuboid_Tet10_mesh(
    const Eigen::Matrix<FLOAT, 3, 1>& size,
    const std::array<int32_t, 3>& elements_per_axis) 
{
    using Vector3S = Eigen::Matrix<FLOAT, 3, 1>;
    FEMMesh<FLOAT> mesh;
    mesh.element_type = FEM_Element::Tet10;
    mesh.connectivity_stride = 10;
    Vector3S min_corner = -0.5 * size;
    Vector3S epa_cast(static_cast<FLOAT>(elements_per_axis[0]), static_cast<FLOAT>(elements_per_axis[1]), static_cast<FLOAT>(elements_per_axis[2]));
    Vector3S delta = size.cwiseQuotient(epa_cast) / 2.0; // Twice the resolution for mid-edge nodes

    int nx = elements_per_axis[0] * 2;
    int ny = elements_per_axis[1] * 2;
    int nz = elements_per_axis[2] * 2;

    auto index = [&](int i, int j, int k) -> int {
        return i * (ny + 1) * (nz + 1) + j * (nz + 1) + k;
        };

    // Generate mesh.vertices including mid-edge nodes
    for (int i = 0; i <= nx; ++i) {
        for (int j = 0; j <= ny; ++j) {
            for (int k = 0; k <= nz; ++k) {
                mesh.vertices.emplace_back(min_corner + Vector3S((FLOAT)i, (FLOAT)j, (FLOAT)k).cwiseProduct(delta));
            }
        }
    }

    // Step 2: Generate the mesh.connectivity using Tet10 elements.
    for (int i = 0; i < nx; i += 2) {
        for (int j = 0; j < ny; j += 2) {
            for (int k = 0; k < nz; k += 2) {

                int indices[8][3] = {
                    {i + 0, j + 0, k + 0},
                    {i + 2, j + 0, k + 0},
                    {i + 2, j + 2, k + 0},
                    {i + 0, j + 2, k + 0},
                    {i + 0, j + 0, k + 2},
                    {i + 2, j + 0, k + 2},
                    {i + 2, j + 2, k + 2},
                    {i + 0, j + 2, k + 2}
                };

                auto v = [&](int a) { return index(indices[a][0], indices[a][1], indices[a][2]); };
                auto m = [&](int a, int b) { return index((indices[a][0] + indices[b][0])/2, (indices[a][1] + indices[b][1])/2, (indices[a][2] + indices[b][2])/2); };
                auto tet10 = [&](std::array<int, 4> t) { return std::array<int, 10>({ v(t[0]), v(t[1]), v(t[2]), v(t[3]), m(t[0], t[1]), m(t[1], t[2]), m(t[0], t[2]), m(t[0], t[3]), m(t[1], t[3]), m(t[2], t[3]) }); };

                // Rule: v0, v1, v2, v3, m01, m12, m02, m03, m13, m23

                // Split the hexahedron into 5 tetrahedra (Tet10 elements)
                std::array<int, 10> tet;
                tet = tet10({ 0, 1, 6, 5 }); mesh.connectivity.insert(mesh.connectivity.end(), tet.begin(), tet.end());
                tet = tet10({ 0, 1, 2, 6 }); mesh.connectivity.insert(mesh.connectivity.end(), tet.begin(), tet.end());
                tet = tet10({ 0, 2, 3, 6 }); mesh.connectivity.insert(mesh.connectivity.end(), tet.begin(), tet.end());
                tet = tet10({ 0, 5, 6, 4 }); mesh.connectivity.insert(mesh.connectivity.end(), tet.begin(), tet.end());
                tet = tet10({ 0, 6, 3, 7 }); mesh.connectivity.insert(mesh.connectivity.end(), tet.begin(), tet.end());
                tet = tet10({ 0, 6, 7, 4 }); mesh.connectivity.insert(mesh.connectivity.end(), tet.begin(), tet.end());
            }
        }
    }
    mesh.n_elements = static_cast<int>(mesh.connectivity.size() / mesh.connectivity_stride);

    reorder_RCM(mesh);
    return mesh;
}
template <typename FLOAT>
FEMMesh<FLOAT> symx::generate_cuboid_Hex8_mesh(
    const Eigen::Matrix<FLOAT, 3, 1>& size,
    const std::array<int32_t, 3>& elements_per_axis) 
{
    using Vector3S = Eigen::Matrix<FLOAT, 3, 1>;
    FEMMesh<FLOAT> mesh;
    mesh.element_type = FEM_Element::Hex8;
    mesh.connectivity_stride = 8;
    Vector3S min_corner = -0.5 * size;
    Vector3S epa_cast(static_cast<FLOAT>(elements_per_axis[0]), static_cast<FLOAT>(elements_per_axis[1]), static_cast<FLOAT>(elements_per_axis[2]));
    Vector3S delta = size.cwiseQuotient(epa_cast);

    for (int i = 0; i <= elements_per_axis[0]; ++i) {
        for (int j = 0; j <= elements_per_axis[1]; ++j) {
            for (int k = 0; k <= elements_per_axis[2]; ++k) {
                mesh.vertices.emplace_back(min_corner + Vector3S((FLOAT)i, (FLOAT)j, (FLOAT)k).cwiseProduct(delta));
            }
        }
    }

    auto index = [&](int i, int j, int k) -> int {
        return i * (elements_per_axis[1] + 1) * (elements_per_axis[2] + 1) +
            j * (elements_per_axis[2] + 1) + k;
        };

    for (int i = 0; i < elements_per_axis[0]; ++i) {
        for (int j = 0; j < elements_per_axis[1]; ++j) {
            for (int k = 0; k < elements_per_axis[2]; ++k) {
                int v0 = index(i, j, k);
                int v1 = index(i + 1, j, k);
                int v2 = index(i + 1, j + 1, k);
                int v3 = index(i, j + 1, k);
                int v4 = index(i, j, k + 1);
                int v5 = index(i + 1, j, k + 1);
                int v6 = index(i + 1, j + 1, k + 1);
                int v7 = index(i, j + 1, k + 1);

                // Add the hexahedron mesh.connectivity
                mesh.connectivity.insert(mesh.connectivity.end(), { v0, v1, v2, v3, v4, v5, v6, v7 });
            }
        }
    }
    mesh.n_elements = static_cast<int>(mesh.connectivity.size() / mesh.connectivity_stride);

    reorder_RCM(mesh);
    return mesh;
}
template <typename FLOAT>
FEMMesh<FLOAT> symx::generate_cuboid_Hex27_mesh(
    const Eigen::Matrix<FLOAT, 3, 1>& size,
    const std::array<int32_t, 3>& elements_per_axis) 
{
    using Vector3S = Eigen::Matrix<FLOAT, 3, 1>;
    FEMMesh<FLOAT> mesh;
    mesh.element_type = FEM_Element::Hex27;
    mesh.connectivity_stride = 27;
    Vector3S min_corner = -0.5 * size;
    Vector3S epa_cast(static_cast<FLOAT>(elements_per_axis[0]), static_cast<FLOAT>(elements_per_axis[1]), static_cast<FLOAT>(elements_per_axis[2]));
    Vector3S delta = size.cwiseQuotient(epa_cast) / 2.0; // Twice the resolution for mid-edge, face center, and cell center nodes

    int nx = elements_per_axis[0] * 2;
    int ny = elements_per_axis[1] * 2;
    int nz = elements_per_axis[2] * 2;

    auto index = [&](int i, int j, int k) -> int {
        return i * (ny + 1) * (nz + 1) + j * (nz + 1) + k;
        };

    // Generate mesh.vertices including mid-edge, face center, and cell center nodes
    for (int i = 0; i <= nx; ++i) {
        for (int j = 0; j <= ny; ++j) {
            for (int k = 0; k <= nz; ++k) {
                mesh.vertices.emplace_back(min_corner + Vector3S((FLOAT)i, (FLOAT)j, (FLOAT)k).cwiseProduct(delta));
            }
        }
    }

    // Step 2: Generate the mesh.connectivity using Hex27 elements.
    for (int i = 0; i < nx; i += 2) {
        for (int j = 0; j < ny; j += 2) {
            for (int k = 0; k < nz; k += 2) {

                int i0 = i + 0;
                int i1 = i + 1;
                int i2 = i + 2;
                int j0 = j + 0;
                int j1 = j + 1;
                int j2 = j + 2;
                int k0 = k + 0;
                int k1 = k + 1;
                int k2 = k + 2;

                mesh.connectivity.insert(mesh.connectivity.end(), 
                    {
                        // Bottom mesh.vertices
                        index(i0, j0, k0),  // 0 1
                        index(i2, j0, k0),
                        index(i2, j2, k0),
                        index(i0, j2, k0),

                        // Top mesh.vertices
                        index(i0, j0, k2),  // 4 5
                        index(i2, j0, k2),
                        index(i2, j2, k2),
                        index(i0, j2, k2),

                        // Bottom mid-edges
                        index(i1, j0, k0),  // 8 9
                        index(i2, j1, k0),
                        index(i1, j2, k0),
                        index(i0, j1, k0),

                        // Top mid-edges
                        index(i1, j0, k2),  // 12 13
                        index(i2, j1, k2),
                        index(i1, j2, k2),
                        index(i0, j1, k2),

                        // Middle mid-edges
                        index(i0, j0, k1),  // 16 17
                        index(i2, j0, k1),
                        index(i2, j2, k1),
                        index(i0, j2, k1),

                        // Faces
                        index(i0, j1, k1),  // 20 21
                        index(i2, j1, k1),
                        index(i1, j2, k1),  // NOTE: Swap this one (22) with the next (23) for VTK export!
                        index(i1, j0, k1),
                        index(i1, j1, k0),
                        index(i1, j1, k2),
                        
                        // Center
                        index(i1, j1, k1)  // 26 27
                    }
                );
            }
        }
    }
    mesh.n_elements = static_cast<int>(mesh.connectivity.size() / mesh.connectivity_stride);

    reorder_RCM(mesh);
    return mesh;
}

template <typename FLOAT>
FEMMesh<FLOAT> symx::generate_cuboid_mesh(
    const FEM_Element& elem, 
    const Eigen::Matrix<FLOAT, 3, 1>& size,
    const std::array<int32_t, 3>& elements_per_axis)
{
    switch (elem)
    {
    case FEM_Element::Tet4:
        return generate_cuboid_Tet4_mesh(size, elements_per_axis);
    case FEM_Element::Tet10:
        return generate_cuboid_Tet10_mesh(size, elements_per_axis);
    case FEM_Element::Hex8:
        return generate_cuboid_Hex8_mesh(size, elements_per_axis);
    case FEM_Element::Hex27:
        return generate_cuboid_Hex27_mesh(size, elements_per_axis);
    default:
        std::cout << "symx error: generate_cuboid_mesh() got unsupported element type." << std::endl;
        exit(-1);
    }
}

template <typename FLOAT>
TriangleMesh<FLOAT> symx::generate_triangle_grid(const Eigen::Matrix<FLOAT, 2, 1>& size, const std::array<int32_t, 2>& quads_per_axis)
{
    using Vector2S = Eigen::Matrix<FLOAT, 2, 1>;
    using Vector3S = Eigen::Matrix<FLOAT, 3, 1>;
    TriangleMesh<FLOAT> mesh;

	// Precomputation
	const int nx = quads_per_axis[0] + 1;
	const int ny = quads_per_axis[1] + 1;
	const int n_points = nx * ny;
	const int n_quadrilaterals = quads_per_axis[0] * quads_per_axis[1];
	const int n_triangles = 2 * n_quadrilaterals;
	const FLOAT dx = size[0] / static_cast<FLOAT>(quads_per_axis[0]);
	const FLOAT dy = size[1] / static_cast<FLOAT>(quads_per_axis[1]);
    const Vector2S bottom = -0.5 * size;
    const FLOAT z = 0.0;

	// Points
	/* Correspond to a quadrilateral following the pattern:
			[[0, 0],
			 [0, 1],
			 [1, 0],
			 [1, 1]]

		move y -> move x
	*/
	mesh.vertices.resize(n_points);
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			const int idx = ny * i + j;
			mesh.vertices[idx] = { bottom[0] + static_cast<FLOAT>(i) * dx, bottom[1] + static_cast<FLOAT>(j) * dy, z };
		}
	}

	// Connectivity
	/* Corresponds to a split quadrilateral. Resulting triangles with
	   homogeneous normals
			[[0, 2, 3],
			 [0, 3, 1]]

		The relation between an element "coordinate" in the grid
		and its vertices global indices is:

			node_idx = ny*(ei + ni) + (ej + nj)

		where n_xyz is the number of nodes in the xyz dimension, e_ijk
		is the element "coordinate" in the ijk (grid) dimension and n_ijk
		is the local node "coordinate" within the hexahedron.
	*/
	mesh.triangles.clear();
	mesh.triangles.reserve(n_triangles);
	for (int ei = 0; ei < quads_per_axis[0]; ei++) {
		for (int ej = 0; ej < quads_per_axis[1]; ej++) {
			int nodes_idx[] = { ny * (ei + 0) + (ej + 0),
							   ny * (ei + 0) + (ej + 1),
							   ny * (ei + 1) + (ej + 0),
							   ny * (ei + 1) + (ej + 1) };

			if (ei % 2 == ej % 2) {
				mesh.triangles.push_back({ nodes_idx[0], nodes_idx[2], nodes_idx[3] });
				mesh.triangles.push_back({ nodes_idx[0], nodes_idx[3], nodes_idx[1] });
			}
			else {
				mesh.triangles.push_back({ nodes_idx[0], nodes_idx[2], nodes_idx[1] });
				mesh.triangles.push_back({ nodes_idx[2], nodes_idx[3], nodes_idx[1] });
			}
		}
	}
	
    reorder_RCM(mesh);
    return mesh;
}

// Explicit instantiations
template FEMMesh<double> symx::generate_cuboid_Tet4_mesh(const Eigen::Vector3d&, const std::array<int32_t, 3>&);
template FEMMesh<double> symx::generate_cuboid_Tet10_mesh(const Eigen::Vector3d&, const std::array<int32_t, 3>&);
template FEMMesh<double> symx::generate_cuboid_Hex8_mesh(const Eigen::Vector3d&, const std::array<int32_t, 3>&);
template FEMMesh<double> symx::generate_cuboid_Hex27_mesh(const Eigen::Vector3d&, const std::array<int32_t, 3>&);
template FEMMesh<double> symx::generate_cuboid_mesh(const symx::FEM_Element&, const Eigen::Vector3d&, const std::array<int32_t, 3>&);
template TriangleMesh<double> symx::generate_triangle_grid(const Eigen::Vector2d&, const std::array<int32_t, 2>&);

template FEMMesh<float> symx::generate_cuboid_Tet4_mesh(const Eigen::Vector3f&, const std::array<int32_t, 3>&);
template FEMMesh<float> symx::generate_cuboid_Tet10_mesh(const Eigen::Vector3f&, const std::array<int32_t, 3>&);
template FEMMesh<float> symx::generate_cuboid_Hex8_mesh(const Eigen::Vector3f&, const std::array<int32_t, 3>&);
template FEMMesh<float> symx::generate_cuboid_Hex27_mesh(const Eigen::Vector3f&, const std::array<int32_t, 3>&);
template FEMMesh<float> symx::generate_cuboid_mesh(const symx::FEM_Element&, const Eigen::Vector3f&, const std::array<int32_t, 3>&);
template TriangleMesh<float> symx::generate_triangle_grid(const Eigen::Vector2f&, const std::array<int32_t, 2>&);
