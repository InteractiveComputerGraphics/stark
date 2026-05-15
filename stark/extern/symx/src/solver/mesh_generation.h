#pragma once

#include <vector>
#include <array>
#include <Eigen/Dense>
#include <cstdint>

#include "fem_integrators.h"

namespace symx 
{
    template <typename FLOAT>
    struct FEMMesh
    {
        FEM_Element element_type;
        std::vector<Eigen::Matrix<FLOAT, 3, 1>> vertices;
        std::vector<int> connectivity;
        int connectivity_stride = -1;
        int n_elements = -1;
    };
    
    template <typename FLOAT>
    struct TriangleMesh
    {
        std::vector<Eigen::Matrix<FLOAT, 3, 1>> vertices;
        std::vector<std::array<int, 3>> triangles;
    };

    // Prepend each element in a flat connectivity array with its 0-based element index,
    // growing the stride by 1.
    std::vector<int> enumerate(const std::vector<int>& connectivity, int stride);

    template <std::size_t STRIDE>
    std::vector<std::array<int, STRIDE>> as_array_vec(const std::vector<int>& connectivity);

    template <typename FLOAT>
    FEMMesh<FLOAT> generate_cuboid_Tet4_mesh(
        const Eigen::Matrix<FLOAT, 3, 1>& size,
        const std::array<int32_t, 3>& elements_per_axis);

    template <typename FLOAT>
    FEMMesh<FLOAT> generate_cuboid_Tet10_mesh(
        const Eigen::Matrix<FLOAT, 3, 1>& size,
        const std::array<int32_t, 3>& elements_per_axis);

    template <typename FLOAT>
    FEMMesh<FLOAT> generate_cuboid_Hex8_mesh(
        const Eigen::Matrix<FLOAT, 3, 1>& size,
        const std::array<int32_t, 3>& elements_per_axis);

    template <typename FLOAT>
    FEMMesh<FLOAT> generate_cuboid_Hex27_mesh(
        const Eigen::Matrix<FLOAT, 3, 1>& size,
        const std::array<int32_t, 3>& elements_per_axis);

    template <typename FLOAT>
    FEMMesh<FLOAT> generate_cuboid_mesh(
        const symx::FEM_Element& elem,
        const Eigen::Matrix<FLOAT, 3, 1>& size,
        const std::array<int32_t, 3>& elements_per_axis);

    template <typename FLOAT>
    TriangleMesh<FLOAT> generate_triangle_grid(
        const Eigen::Matrix<FLOAT, 2, 1>& size,
        const std::array<int32_t, 2>& quads_per_axis);


	// DEFINITIONS ==========================================================================================
    template <std::size_t STRIDE>
    std::vector<std::array<int, STRIDE>> as_array_vec(const std::vector<int> &connectivity)
    {
        const int n_elements = static_cast<int>(connectivity.size() / STRIDE);
        std::vector<std::array<int, STRIDE>> out;
        out.reserve(n_elements);
        for (int i = 0; i < n_elements; ++i) {
            std::array<int, STRIDE> elem;
            std::copy(connectivity.begin() + i * STRIDE, connectivity.begin() + (i + 1) * STRIDE, elem.begin());
            out.push_back(elem);
        }
        return out;
    }

}
