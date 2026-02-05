#pragma once
#include <functional>
#include <cstdint>
#include <vector>
#include <array>
#include <iostream>
#include <type_traits>
#include <Eigen/Dense>

namespace symx
{
    // Alias for Eigen Row-Major Matrix. SymX does not support Column-Major matrices.
    template<typename T>
	using EigenMatrixRM = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;


    /*
     * ConnectivityMap
     * 
     * Purpose:
     *   Defines user's connectivity data for loop evaluations.
     *   Uses lambda functions (instead of raw pointers) to allow resizing of the data.
     *   Supports various data sources out-of-the-box: flat arrays, std::vector of arrays, Eigen matrices.
     *   Otherwise, users can provide custom lambdas to bind their own data structures.
     * 
     * Warning:
     *   The data must be contiguous in memory and row-major ordered.
     */
    struct ConnectivityMap
    {
        std::function<const int32_t*()> data = nullptr; // Lambda to obtain pointer to the beginning
        std::function<int32_t()> n_elements = nullptr;  // Lambda to obtain current number of elements (e.g. triangles)
        int32_t stride = -1;                            // Stride in the array (e.g. 3 for triangles, 4 for quads)

        ConnectivityMap() = default;

        // Generic constructor
        ConnectivityMap(std::function<const int32_t*()> data, std::function<int32_t()> n_elements, int32_t stride)
            : data(data), n_elements(n_elements), stride(stride)
        {}

        // Flat vector constructor (e.g. triangle indices flattened)
        ConnectivityMap(const std::vector<int32_t>& arr, int32_t stride)
            : stride(stride)
        {
            this->data = [&arr]() { return arr.data(); };
            this->n_elements = [&arr, stride]() { return (int32_t)(arr.size() / stride); };
        }

        // List of Arrays constructor (e.g. std::vector<std::array<int, 3>>)
        template<std::size_t N>
        ConnectivityMap(const std::vector<std::array<int32_t, N>>& arr)
        {
            this->stride = (int32_t)N;
            this->data = [&arr]() { return (const int32_t*)arr[0].data(); };
            this->n_elements = [&arr]() { return (int32_t)arr.size(); };
        }

        // Eigen Matrix constructor
        ConnectivityMap(const EigenMatrixRM<int32_t>& arr)
        {
            this->stride = (int32_t)arr.cols();
            this->data = [&arr]() { return arr.data(); };
            this->n_elements = [&arr]() { return (int32_t)arr.rows(); };
        }
    };

    /*
     * DataMap<T>
     *
     * Purpose:
     *   Binds symbolic variables (inputs or DOFs) to actual memory locations.
     *   It abstracts away the storage layout (std::vector, Eigen, flat arrays, etc.)
     *   so the solver/evaluator can access data uniformly.
     *   Uses lambda functions (instead of raw pointers) to allow resizing of the data.
     *
     * Warning:
     *   The data must be contiguous in memory and row-major ordered.
     * 
     * Note:
     *  Specialization for different types is delegated to consumer classes (e.g. MappedWorkspace)
     *  because handling different containers and const vs non-const data get complex.
     *  Untangling those with templates is far less readable than letting other classes handle it.
     */
    template<typename T>
    struct DataMap
    {
        std::function<std::uintptr_t()> id = nullptr;  // Unique identifier: address of the data source container (e.g. &std::vector, &Eigen::Matrix). Used for identity checks.
        std::function<T*()> data = nullptr;            // Lambda to obtain pointer to the beginning
        std::function<int32_t()> n_elements = nullptr; // Lambda to obtain current number of elements (e.g. vertices)
        int32_t stride = -1;                           // Stride in the array (e.g. 3 for 3D vectors)
        int32_t connectivity_index = -1;               // Entry idx in the connectivity array the symbol is associated with. -1 for no connectivity (fixed) values
        int32_t first_symbol_idx = -1;                 // Location of the first symbol in the global symbols (an input) array

        DataMap() = default;
        DataMap(std::function<std::uintptr_t()> id, std::function<T*()> data, std::function<int32_t()> n_elements, int32_t stride, int32_t connectivity_index, int32_t first_symbol_idx)
            : id(id), data(data), n_elements(n_elements), stride(stride), connectivity_index(connectivity_index), first_symbol_idx(first_symbol_idx)
        {
            if (!id) {
                std::cout << "symx error: DataMap id must not be null. All data sources must provide a unique identifier." << std::endl;
                exit(-1);
            }
        }
        inline int32_t size() const { return n_elements() * stride; }
    };
}
