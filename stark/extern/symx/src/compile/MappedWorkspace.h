#pragma once
#include <functional>
#include <cstdint>
#include <vector>
#include <array>
#include <iostream>
#include <memory>
#include <Eigen/Dense>

#include "../symbol/Workspace.h"
#include "Element.h"
#include "LabelledConnectivity.h"
#include "View.h"
#include "data_maps.h"


namespace symx
{
    template<typename FLOAT>
    struct Summation
    {
        std::vector<FLOAT> data;
        int32_t n_iterations = -1;
        int32_t stride = -1;
        int32_t first_symbol_idx = -1;
    };

	/*
	 * MappedWorkspace: Binds a symbolic Workspace to actual data for loop-based evaluation.
	 * 
	 * Key concepts:
	 *   - Workspace (ws): Contains symbolic expressions (Scalar, Vector, Matrix)
	 *   - DataMaps (maps): Bind symbols to actual memory via lambdas. Each map tracks:
	 *       - id: container address for identity matching across contexts
	 *       - data: pointer to actual memory
	 *       - n_elements: number of rows/items in the array
	 *       - stride: elements per row (e.g., 3 for Vec3)
	 *       - connectivity_index: which connectivity column indexes this data (-1 for fixed/global data)
	 *       - first_symbol_idx: where in the symbol array this binding starts
	 *   - ConnectivityMap (conn): Defines which elements (triangles, edges, etc.) to iterate over
	 *   - Summation: Optional mechanism for summing over a vector of values e.g. quadrature points
	 * 
	 * This class does not evaluate - it's passed to CompiledInLoop for actual evaluation.
	 */
    template<typename FLOAT>
    class MappedWorkspace
    {
    public:
        /* Fields */
        Workspace ws;                            // Symbolic expressions
        Summation<FLOAT> summation;              // Optional: data for summation loops
        std::vector<DataMap<const FLOAT>> maps;  // Bindings: symbols -> actual data (const because inputs are read-only)
		ConnectivityMap conn;                    // Connectivity: defines loop iteration structure
		std::string name = "NO_NAME";            // Debug name for error messages

        /* Methods */
		MappedWorkspace(ConnectivityMap conn);
		static inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> create(ConnectivityMap conn);
		static inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> create(std::function<const int32_t*()> data, std::function<int32_t()> n_elements, const int32_t stride);
		static inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> create(const std::vector<int32_t>& arr, const int32_t stride);
		template<std::size_t N>
		static inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> create(const std::vector<std::array<int32_t, N>>& arr);
		static inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> create(const EigenMatrixRM<int32_t>& arr);
		template<std::size_t N>
		static inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> create(const LabelledConnectivity<N>& conn);
		void set_name(const std::string& name);

		// Make fixed data (no indexing)
		inline Scalar make_scalar(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, const std::string& name = "");
		inline Scalar make_scalar(const FLOAT& scalar, const std::string& name = "");
		inline Vector make_vector(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, const int32_t stride, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline Vector make_vector(const STATIC_VECTOR& arr, const std::string& name = "");
		inline Matrix make_matrix(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, const std::array<int, 2> shape, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline Matrix make_matrix(const DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const std::string& name = "");

		// Make Scalar
		inline Scalar make_scalar(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const Index& idx, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline Scalar make_scalar(const DYNAMIC_VECTOR& arr, const Index& idx, const std::string& name = "");

		// Make Scalars
		inline std::vector<Scalar> make_scalars(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const std::vector<Index>& indices, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline std::vector<Scalar> make_scalars(const DYNAMIC_VECTOR& arr, const std::vector<Index>& indices, const std::string& name = "");

		// Make Vector
		inline Vector make_vector(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const int32_t stride, const Index& idx, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline Vector make_vector(const std::vector<STATIC_VECTOR>& arr, const Index& idx, const std::string& name = "");
		inline Vector make_vector(const EigenMatrixRM<FLOAT>& arr, const Index& idx, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline Vector make_vector(const DYNAMIC_VECTOR& arr, const int32_t stride, const Index& idx, const std::string& name = "");

		// Make Vectors
		inline std::vector<Vector> make_vectors(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const int32_t stride, const std::vector<Index>& indices, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline std::vector<Vector> make_vectors(const std::vector<STATIC_VECTOR>& arr, const std::vector<Index>& indices, const std::string& name = "");
		inline std::vector<Vector> make_vectors(const EigenMatrixRM<FLOAT>& arr, const std::vector<Index>& indices, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline std::vector<Vector> make_vectors(const std::vector<STATIC_VECTOR>& arr, const Element& element, const std::string& name = "");
		inline std::vector<Vector> make_vectors(const EigenMatrixRM<FLOAT>& arr, const Element& element, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline std::vector<Vector> make_vectors(const DYNAMIC_VECTOR& arr, const int32_t stride, const std::vector<Index>& indices, const std::string& name = "");

		// Make Matrix
		inline Matrix make_matrix(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const std::array<int, 2> shape, const Index& idx, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline Matrix make_matrix(const std::vector<STATIC_VECTOR>& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name = "");
		inline Matrix make_matrix(const EigenMatrixRM<FLOAT>& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline Matrix make_matrix(const DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name = "");

		// Make Matrices
		inline std::vector<Matrix> make_matrices(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline std::vector<Matrix> make_matrices(const std::vector<STATIC_VECTOR>& arr, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name = "");
		inline std::vector<Matrix> make_matrices(const EigenMatrixRM<FLOAT>& arr, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline std::vector<Matrix> make_matrices(const DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name = "");
		
		// Make summation vector
		inline Vector make_summation_vector(const FLOAT* data, const int32_t stride, const int32_t n_iterations);
		template<typename STATIC_VECTOR>
		inline Vector make_summation_vector(const std::vector<STATIC_VECTOR>& iteration_vectors);
		template<typename STATIC_VECTOR>
		inline Scalar add_for_each(const std::vector<STATIC_VECTOR>& iteration_vectors, std::function<Scalar(Vector& vec)> f);
		inline Scalar add_for_each(const EigenMatrixRM<FLOAT>& iteration_vectors, std::function<Scalar(Vector& vec)> f);
		inline bool has_summation() const;

		// Make constants
		inline Scalar make_zero();
		inline Scalar make_one();
		inline Vector make_zero_vector(const int32_t size);
		inline Matrix make_zero_matrix(const std::array<int32_t, 2> shape);
		inline Matrix make_identity_matrix(const int32_t size);

		// Connectivity
		inline int32_t get_n_elements() const;
		inline int32_t get_connectivity_stride() const;
		inline const int32_t* get_connectivity_data() const;
		inline View<int32_t> get_connectivity_elem(const int32_t i) const;
		
		// Verification
		inline void verify_data_maps() const;
		inline void verify_no_out_of_bounds() const;
		
		// Misc
		inline const Workspace& get_workspace() const;
		std::vector<Scalar> get_symbols(const DataMap<double>& data_map);
		std::vector<DataMap<const FLOAT>> get_original_maps(const DataMap<double>& data_map);

	private:
		inline void _check_Eigen_stride_inferrable(const EigenMatrixRM<FLOAT>& arr, const std::string& operation_description) const;
    };
	// =========================================================================================================================
	// =========================================================================================================================
	
    // ============================================================================
    // Constructors
    // ============================================================================
    template <typename FLOAT>
    inline MappedWorkspace<FLOAT>::MappedWorkspace(ConnectivityMap conn)
        : conn(conn)
    {
    }

    template <typename FLOAT>
    inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> MappedWorkspace<FLOAT>::create(ConnectivityMap conn)
    {
        auto mws = std::make_shared<MappedWorkspace<FLOAT>>(conn);
		return {mws, Element(conn.stride)};
    }

    template <typename FLOAT>
    inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> MappedWorkspace<FLOAT>::create(std::function<const int32_t *()> data, std::function<int32_t()> n_elements, const int32_t stride)
    {
        auto mws = std::make_shared<MappedWorkspace<FLOAT>>(ConnectivityMap(data, n_elements, stride));
		return {mws, Element(stride)};
    }

    template <typename FLOAT>
    inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> MappedWorkspace<FLOAT>::create(const std::vector<int32_t> &arr, const int32_t stride)
    {
        auto mws = std::make_shared<MappedWorkspace<FLOAT>>(ConnectivityMap(arr, stride));
		return {mws, Element(stride)};
    }

    template <typename FLOAT>
	template<std::size_t N>
    inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> MappedWorkspace<FLOAT>::create(const std::vector<std::array<int32_t, N>>& arr)
    {
        auto mws = std::make_shared<MappedWorkspace<FLOAT>>(ConnectivityMap(arr));
		return {mws, Element((int32_t)N)};
    }

    template <typename FLOAT>
    inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> MappedWorkspace<FLOAT>::create(const EigenMatrixRM<int32_t> &arr)
    {
        auto mws = std::make_shared<MappedWorkspace<FLOAT>>(ConnectivityMap(arr));
		return {mws, Element((int32_t)arr.cols())};
    }

    template <typename FLOAT>
	template<std::size_t N>
    inline std::pair<std::shared_ptr<MappedWorkspace<FLOAT>>, Element> MappedWorkspace<FLOAT>::create(const LabelledConnectivity<N>& conn)
    {
        auto mws = std::make_shared<MappedWorkspace<FLOAT>>(ConnectivityMap(conn.data));
		return {mws, conn.get_element()};
    }

	// ============================================================================
    // Make fixed data (no indexing)
    // Fixed data: connectivity_index=-1 means the value is the same for all elements
    // (e.g., global parameters like time step size)
    // ============================================================================
	template<typename FLOAT>
	inline Scalar MappedWorkspace<FLOAT>::make_scalar(std::function<std::uintptr_t()> id, std::function<const FLOAT* ()> data, const std::string& name)
	{
		this->maps.emplace_back(
			/*id=*/ id,
			/*data=*/ data, 
			/*n_elements=*/ []() { return 1; }, 
			/*stride=*/ 1,
			/*connectivity_index=*/ -1,
			/*first_symbol_idx=*/ this->ws.get_n_symbols());
		return this->ws.make_scalar(name);
	}
	template<typename FLOAT>
	inline Scalar MappedWorkspace<FLOAT>::make_scalar(const FLOAT& scalar, const std::string& name)
	{
		return this->make_scalar(
			[&scalar]() { return reinterpret_cast<std::uintptr_t>(&scalar); },
			[&scalar]() { return &scalar; }, 
			name);
	}
	template<typename FLOAT>
	inline Vector MappedWorkspace<FLOAT>::make_vector(std::function<std::uintptr_t()> id, std::function<const FLOAT*()> data, const int32_t stride, const std::string& name)
	{
		this->maps.emplace_back(
			/*id=*/ id,
			/*data=*/ data, 
			/*n_elements=*/ []() { return 1; }, 
			/*stride=*/ stride, 
			/*connectivity_index=*/ -1,
			/*first_symbol_idx=*/ this->ws.get_n_symbols());
		return this->ws.make_vector(name, stride);
	}
	template<typename FLOAT>
	template<typename STATIC_VECTOR>
	inline Vector MappedWorkspace<FLOAT>::make_vector(const STATIC_VECTOR& arr, const std::string& name)
	{
		return this->make_vector(
			[&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
			[&arr]() { return arr.data(); }, 
			(int32_t)arr.size(), 
			name);
	}
	template<typename FLOAT>
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(std::function<std::uintptr_t()> id, std::function<const FLOAT*()> data, const std::array<int, 2> shape, const std::string& name)
	{
		this->maps.emplace_back(
			/*id=*/ id,
			/*data=*/ data, 
			/*n_elements=*/ []() { return 1; }, 
			/*stride=*/ shape[0] * shape[1], 
			/*connectivity_index=*/ -1,
			/*first_symbol_idx=*/ this->ws.get_n_symbols());
		return this->ws.make_matrix(name, shape);
	}
	template<typename FLOAT>
	template<typename DYNAMIC_VECTOR>
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(const DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const std::string& name)
	{
		return this->make_matrix(
			[&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
			std::function<const FLOAT*()>([&arr]() { return arr.data(); }), 
			shape, 
			name);
	}

	// ============================================================================
	// Make Scalar(s) - Indexed by connectivity
	// Indexed data: connectivity_index=idx.idx tells which column of the connectivity
	// array provides the index into this data array.
	// E.g., for triangles [v0,v1,v2], idx=0 means use v0 to index into data.
	// ============================================================================
	template<typename FLOAT>
	inline Scalar MappedWorkspace<FLOAT>::make_scalar(std::function<std::uintptr_t()> id, std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const Index& idx, const std::string& name)
	{
		this->maps.emplace_back(id, data, n_elements, 1, idx.idx, this->ws.get_n_symbols());
		return this->ws.make_scalar(name);
	}
	template<typename FLOAT>
	template<typename DYNAMIC_VECTOR>
	inline Scalar MappedWorkspace<FLOAT>::make_scalar(const DYNAMIC_VECTOR& arr, const Index& idx, const std::string& name)
	{
		return this->make_scalar(
			[&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
			[&arr]() { return arr.data(); }, 
			[&arr]() { return (int32_t)arr.size(); }, 
			idx, 
			name);
	}
    template <typename FLOAT>
    inline std::vector<Scalar> MappedWorkspace<FLOAT>::make_scalars(std::function<std::uintptr_t()> id, std::function<const FLOAT *()> data, std::function<int32_t()> n_elements, const std::vector<Index> &indices, const std::string& name)
    {
        std::vector<Scalar> scalars;
        for (const auto &idx : indices) {
            scalars.push_back(this->make_scalar(id, data, n_elements, idx, name));
        }
        return scalars;
    }

    template <typename FLOAT>
    template <typename DYNAMIC_VECTOR>
    inline std::vector<Scalar> MappedWorkspace<FLOAT>::make_scalars(const DYNAMIC_VECTOR &arr, const std::vector<Index> &indices, const std::string& name)
    {
        std::vector<Scalar> scalars;
        for (const auto &idx : indices) {
            scalars.push_back(this->make_scalar(arr, idx, name));
        }
        return scalars;
    }

	// ============================================================================
	// Make Vector(s) - Indexed by connectivity
	// For vectors (stride > 1), data is laid out as: [v0_x, v0_y, v0_z, v1_x, v1_y, v1_z, ...]
	// Accessing element i means accessing data[i*stride : i*stride+stride]
	// ============================================================================
	template<typename FLOAT>
	inline Vector MappedWorkspace<FLOAT>::make_vector(std::function<std::uintptr_t()> id, std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const int32_t stride, const Index& idx, const std::string& name)
	{
		this->maps.emplace_back(id, data, n_elements, stride, idx.idx, this->ws.get_n_symbols());
		return this->ws.make_vector(name, stride);
	}
    template <typename FLOAT>
    inline Vector MappedWorkspace<FLOAT>::make_vector(const EigenMatrixRM<FLOAT> &arr, const Index &idx, const std::string& name)
    {
		this->_check_Eigen_stride_inferrable(arr, "in make_vector");
		return this->make_vector(
			[&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
			[&arr]() { return arr.data(); },
			[&arr]() { return (int32_t)arr.rows(); },
			(int32_t)arr.cols(),
			idx,
			name);
    }
    template<typename FLOAT>
	template<typename STATIC_VECTOR>
	inline Vector MappedWorkspace<FLOAT>::make_vector(const std::vector<STATIC_VECTOR>& arr, const Index& idx, const std::string& name)
	{
		constexpr int32_t stride = sizeof(STATIC_VECTOR) / sizeof(FLOAT);
		return this->make_vector(
			[&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
			[&arr]() { return arr[0].data(); },
			[&arr]() { return (int32_t)arr.size(); },
			stride,
			idx,
			name);
	}
	template<typename FLOAT>
	template<typename DYNAMIC_VECTOR>
	inline Vector MappedWorkspace<FLOAT>::make_vector(const DYNAMIC_VECTOR& arr, const int32_t stride, const Index& idx, const std::string& name)
	{
		return this->make_vector(
			[&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
			[&arr]() { return arr.data(); },
			[&arr, stride]() { return (int32_t)arr.size()/stride; },
			stride,
			idx,
			name);
	}

	template<typename FLOAT>
	inline std::vector<Vector> MappedWorkspace<FLOAT>::make_vectors(std::function<std::uintptr_t()> id, std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const int32_t stride, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Vector> vectors;
		for (size_t i = 0; i < indices.size(); i++) {
			vectors.push_back(this->make_vector(id, data, n_elements, stride, indices[i], name + std::to_string(i)));
		}
		return vectors;
	}

	template<typename FLOAT>
	template<typename STATIC_VECTOR>
	inline std::vector<Vector> MappedWorkspace<FLOAT>::make_vectors(const std::vector<STATIC_VECTOR>& arr, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Vector> vectors;
		for (size_t i = 0; i < indices.size(); i++) {
			vectors.push_back(this->make_vector(arr, indices[i], name + std::to_string(i)));
		}
		return vectors;
	}

	template<typename FLOAT>
	inline std::vector<Vector> MappedWorkspace<FLOAT>::make_vectors(const EigenMatrixRM<FLOAT>& arr, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Vector> vectors;
		for (size_t i = 0; i < indices.size(); i++) {
			vectors.push_back(this->make_vector(arr, indices[i], name + std::to_string(i)));
		}
		return vectors;
	}

	template<typename FLOAT>
	template<typename STATIC_VECTOR>
	inline std::vector<Vector> MappedWorkspace<FLOAT>::make_vectors(const std::vector<STATIC_VECTOR>& arr, const Element& element, const std::string& name)
	{
		return this->make_vectors(arr, element.all(), name);
	}

	template<typename FLOAT>
	inline std::vector<Vector> MappedWorkspace<FLOAT>::make_vectors(const EigenMatrixRM<FLOAT>& arr, const Element& element, const std::string& name)
	{
		return this->make_vectors(arr, element.all(), name);
	}

	template<typename FLOAT>
	template<typename DYNAMIC_VECTOR>
	inline std::vector<Vector> MappedWorkspace<FLOAT>::make_vectors(const DYNAMIC_VECTOR& arr, const int32_t stride, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Vector> vectors;
		for (size_t i = 0; i < indices.size(); i++) {
			vectors.push_back(this->make_vector(arr, stride, indices[i], name + std::to_string(i)));
		}
		return vectors;
	}

	// ============================================================================
	// Make Matrix/Matrices
	// ============================================================================
	template<typename FLOAT>
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(std::function<std::uintptr_t()> id, std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const std::array<int, 2> shape, const Index& idx, const std::string& name)
	{
		this->maps.emplace_back(id, data, n_elements, shape[0] * shape[1], idx.idx, this->ws.get_n_symbols());
		return this->ws.make_matrix(name, shape);
	}
	template<typename FLOAT>
	template<typename STATIC_VECTOR>
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(const std::vector<STATIC_VECTOR>& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name)
	{
		constexpr int32_t expected_size = sizeof(STATIC_VECTOR) / sizeof(FLOAT);
		if (expected_size != shape[0] * shape[1]) {
			const std::string name_prefix = this->name.empty() ? "" : "\"" + this->name + "\" ";
			std::cout << "symx error: MappedWorkspace " << name_prefix
					  << "matrix \"" << name << "\" shape [" << shape[0] << "," << shape[1] 
					  << "] requires " << shape[0] * shape[1] << " elements but STATIC_VECTOR has " 
					  << expected_size << " elements" << std::endl;
			exit(-1);
		}
		return this->make_matrix(
			[&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
			[&arr]() { return arr[0].data(); },
			[&arr]() { return (int32_t)arr.size(); },
			shape,
			idx,
			name);
	}
	template<typename FLOAT>
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(const EigenMatrixRM<FLOAT>& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name)
	{
		this->_check_Eigen_stride_inferrable(arr, "when creating matrix \"" + name + "\"");
		return this->make_matrix(
			[&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
			[&arr]() { return arr.data(); },
			[&arr]() { return (int32_t)arr.rows(); },
			shape,
			idx,
			name);
	}
	template<typename FLOAT>
	template<typename DYNAMIC_VECTOR>
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(const DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name)
	{
		return this->make_matrix(
			[&arr]() { return reinterpret_cast<std::uintptr_t>(&arr); },
			[&arr]() { return arr.data(); },
			[&arr, shape]() { return (int32_t)arr.size() / (shape[0] * shape[1]); },
			shape,
			idx,
			name);
	}

	template<typename FLOAT>
	inline std::vector<Matrix> MappedWorkspace<FLOAT>::make_matrices(std::function<std::uintptr_t()> id, std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Matrix> matrices;
		for (size_t i = 0; i < indices.size(); i++) {
			matrices.push_back(this->make_matrix(id, data, n_elements, shape, indices[i], name + std::to_string(i)));
		}
		return matrices;
	}

	template<typename FLOAT>
	template<typename STATIC_VECTOR>
	inline std::vector<Matrix> MappedWorkspace<FLOAT>::make_matrices(const std::vector<STATIC_VECTOR>& arr, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Matrix> matrices;
		for (size_t i = 0; i < indices.size(); i++) {
			matrices.push_back(this->make_matrix(arr, shape, indices[i], name + std::to_string(i)));
		}
		return matrices;
	}

	template<typename FLOAT>
	inline std::vector<Matrix> MappedWorkspace<FLOAT>::make_matrices(const EigenMatrixRM<FLOAT>& arr, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Matrix> matrices;
		for (size_t i = 0; i < indices.size(); i++) {
			matrices.push_back(this->make_matrix(arr, shape, indices[i], name + std::to_string(i)));
		}
		return matrices;
	}

	template<typename FLOAT>
	template<typename DYNAMIC_VECTOR>
	inline std::vector<Matrix> MappedWorkspace<FLOAT>::make_matrices(const DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Matrix> matrices;
		for (size_t i = 0; i < indices.size(); i++) {
			matrices.push_back(this->make_matrix(arr, shape, indices[i], name + std::to_string(i)));
		}
		return matrices;
	}

	// ============================================================================
	// Make summation vector
	// ============================================================================
	template<typename FLOAT>
	inline Vector MappedWorkspace<FLOAT>::make_summation_vector(const FLOAT* data, const int32_t stride, const int32_t n_iterations)
	{
		// Check if summation is already set
		if (this->has_summation()) {
			const std::string name_prefix = this->name.empty() ? "" : "\"" + this->name + "\" ";
			std::cout << "symx error: MappedWorkspace " << name_prefix 
					  << "summation already set - only one summation per workspace is allowed" << std::endl;
			exit(-1);
		}

		// Validate inputs
		if (stride <= 0) {
			std::cout << "symx error: MappedWorkspace::make_summation_vector() got invalid stride: " << stride << std::endl;
			exit(-1);
		}
		if (n_iterations <= 0) {
			std::cout << "symx error: MappedWorkspace::make_summation_vector() got invalid n_iterations: " << n_iterations << std::endl;
			exit(-1);
		}

		// Create the vector symbol
		Vector vector = this->ws.make_vector("summation", stride);

        // Set the summation data
		this->summation.stride = stride;
		this->summation.n_iterations = n_iterations;
		this->summation.first_symbol_idx = this->ws.get_n_symbols() - stride;
		this->summation.data.resize(stride * n_iterations);
        std::memcpy(this->summation.data.data(), data, sizeof(FLOAT) * stride * n_iterations);
		
		return vector;
	}

    template <typename FLOAT>
    inline bool MappedWorkspace<FLOAT>::has_summation() const
    {
        return this->summation.n_iterations > 0;
    }

    // Convenience method for vector of arrays
	template<typename FLOAT>
	template<typename STATIC_VECTOR>
	inline Vector MappedWorkspace<FLOAT>::make_summation_vector(const std::vector<STATIC_VECTOR>& iteration_vectors)
	{
		constexpr int32_t stride = sizeof(STATIC_VECTOR) / sizeof(FLOAT);
		const int32_t n_iterations = (int32_t)iteration_vectors.size();
		return this->make_summation_vector(iteration_vectors[0].data(), stride, n_iterations);
	}

    template <typename FLOAT>
    template <typename STATIC_VECTOR>
    inline Scalar MappedWorkspace<FLOAT>::add_for_each(const std::vector<STATIC_VECTOR> &iteration_vectors, std::function<Scalar(Vector &vec)> f)
    {
		Vector summation_vector = this->make_summation_vector(iteration_vectors);
		return f(summation_vector);
    }

    template <typename FLOAT>
    inline Scalar MappedWorkspace<FLOAT>::add_for_each(const EigenMatrixRM<FLOAT> &iteration_vectors, std::function<Scalar(Vector &vec)> f)
    {
		this->_check_Eigen_stride_inferrable(iteration_vectors, "in add_for_each");
		const int32_t stride = (int32_t)iteration_vectors.cols();
		const int32_t n_iterations = (int32_t)iteration_vectors.rows();
		Vector summation_vector = this->make_summation_vector(iteration_vectors.data(), stride, n_iterations);
		return f(summation_vector);
    }

    // ============================================================================
	// Make constants
	// ============================================================================
	template<typename FLOAT>
	inline Scalar MappedWorkspace<FLOAT>::make_zero()
	{
		return this->ws.get_zero();
	}

	template<typename FLOAT>
	inline Scalar MappedWorkspace<FLOAT>::make_one()
	{
		return this->ws.get_one();
	}

	template<typename FLOAT>
	inline Vector MappedWorkspace<FLOAT>::make_zero_vector(const int32_t size)
	{
		return this->ws.get_zero_vector(size);
	}

	template<typename FLOAT>
	inline Matrix MappedWorkspace<FLOAT>::make_zero_matrix(const std::array<int32_t, 2> shape)
	{
		return this->ws.get_zero_matrix(shape);
	}

	template<typename FLOAT>
	inline Matrix MappedWorkspace<FLOAT>::make_identity_matrix(const int32_t size)
	{
		return this->ws.get_identity_matrix(size);
	}

	// ============================================================================
	// Verification
	// ============================================================================
	template<typename FLOAT>
	inline int32_t MappedWorkspace<FLOAT>::get_n_elements() const
	{
		return this->conn.n_elements();
	}

	template<typename FLOAT>
	inline int32_t MappedWorkspace<FLOAT>::get_connectivity_stride() const
	{
		return this->conn.stride;
	}

	template<typename FLOAT>
	inline const int32_t* MappedWorkspace<FLOAT>::get_connectivity_data() const
	{
		return this->conn.data();
	}

	template<typename FLOAT>
	inline View<int32_t> MappedWorkspace<FLOAT>::get_connectivity_elem(const int32_t i) const
	{
		return View<int32_t>(this->conn.data() + this->conn.stride*i, this->conn.stride);
	}

    template <typename FLOAT>
    inline const Workspace &MappedWorkspace<FLOAT>::get_workspace() const
    {
        return this->ws;
    }

	// ============================================================================
	// Verification
	// ============================================================================
	template<typename FLOAT>
	inline void MappedWorkspace<FLOAT>::verify_data_maps() const
	{
		const std::string name_prefix = this->name.empty() ? "" : "\"" + this->name + "\" ";
		const int32_t n_symbols = this->ws.get_n_symbols();

		// Check connectivity if it's set
		if (this->conn.stride < 0) {
			std::cout << "symx error: MappedWorkspace " << name_prefix << "connectivity stride=" << this->conn.stride << " is invalid" << std::endl;
		}
		else {
			if (!this->conn.data) {
				std::cout << "symx error: MappedWorkspace " << name_prefix << "connectivity data lambda is null" << std::endl;
				exit(-1);
			}
			if (!this->conn.n_elements) {
				std::cout << "symx error: MappedWorkspace " << name_prefix << "connectivity n_elements lambda is null" << std::endl;
				exit(-1);
			}
			if (this->conn.stride <= 0) {
				std::cout << "symx error: MappedWorkspace " << name_prefix << "connectivity stride=" << this->conn.stride << " is invalid" << std::endl;
				exit(-1);
			}
		}

		// Check all symbol data maps
		for (size_t map_idx = 0; map_idx < this->maps.size(); map_idx++) {
			const DataMap<const FLOAT>& map = this->maps[map_idx];

			// Get symbol name from first symbol
			std::string symbol_name = "<unknown>";
			if (map.first_symbol_idx >= 0 && map.first_symbol_idx < n_symbols) {
				symbol_name = this->ws.get_expression_graph().get_symbol_names()[map.first_symbol_idx];
			}

			// Check data lambda
			if (!map.data) {
				std::cout << "symx error: MappedWorkspace " << name_prefix << "symbol \"" << symbol_name 
						  << "\" with range [" << map.first_symbol_idx << "+" << map.stride << "]/" << n_symbols 
						  << " data lambda is null" << std::endl;
				exit(-1);
			}

			// Check stride
			if (map.stride < 0) {
				std::cout << "symx error: MappedWorkspace " << name_prefix << "symbol \"" << symbol_name 
						  << "\" with range [" << map.first_symbol_idx << "+" << map.stride << "]/" << n_symbols 
						  << " has invalid stride=" << map.stride << std::endl;
				exit(-1);
			}

			// Check first_symbol_idx
			if (map.first_symbol_idx < 0) {
				std::cout << "symx error: MappedWorkspace " << name_prefix << "symbol \"" << symbol_name 
						  << "\" has invalid first_symbol_idx=" << map.first_symbol_idx << std::endl;
				exit(-1);
			}
			if (map.first_symbol_idx >= n_symbols) {
				std::cout << "symx error: MappedWorkspace " << name_prefix << "symbol \"" << symbol_name 
						  << "\" first_symbol_idx=" << map.first_symbol_idx << " exceeds n_symbols=" << n_symbols << std::endl;
				exit(-1);
			}

			// For connectivity-indexed maps
			if (map.connectivity_index >= 0) {
				// Must have n_elements lambda
				if (!map.n_elements) {
					std::cout << "symx error: MappedWorkspace " << name_prefix << "symbol \"" << symbol_name 
							  << "\" with range [" << map.first_symbol_idx << "+" << map.stride << "]/" << n_symbols 
							  << " has connectivity_index=" << map.connectivity_index << " but n_elements lambda is null" << std::endl;
					exit(-1);
				}

				if (map.connectivity_index >= this->conn.stride) {
					std::cout << "symx error: MappedWorkspace " << name_prefix << "symbol \"" << symbol_name 
							  << "\" with range [" << map.first_symbol_idx << "+" << map.stride << "]/" << n_symbols 
							  << " connectivity_index=" << map.connectivity_index << " exceeds connectivity stride=" << this->conn.stride << std::endl;
					exit(-1);
				}
			}
			// For fixed (non-connectivity) maps
			else if (map.connectivity_index == -1) {
				// Fixed values should have stride > 0 (0 is invalid for any map)
				if (map.stride == 0) {
					std::cout << "symx error: MappedWorkspace " << name_prefix << "symbol \"" << symbol_name 
							  << "\" with range [" << map.first_symbol_idx << "+" << map.stride << "]/" << n_symbols 
							  << " is fixed but has stride=0" << std::endl;
					exit(-1);
				}
			}
			else {
				std::cout << "symx error: MappedWorkspace " << name_prefix << "symbol \"" << symbol_name 
						  << "\" with range [" << map.first_symbol_idx << "+" << map.stride << "]/" << n_symbols 
						  << " has invalid connectivity_index=" << map.connectivity_index << std::endl;
				exit(-1);
			}
		}
	}
    template <typename FLOAT>
    inline void MappedWorkspace<FLOAT>::verify_no_out_of_bounds() const
    {
		const std::string name_prefix = this->name.empty() ? "" : "\"" + this->name + "\" ";
		const int32_t n_symbols = this->ws.get_n_symbols();

		// If no connectivity is set, nothing to verify
		if (this->conn.stride <= 0) {
			return;
		}

		// Get connectivity data
		const int32_t* conn_data = this->conn.data();
		const int32_t n_elements = this->conn.n_elements();
		
		if (!conn_data || n_elements <= 0) {
			return; // Nothing to check
		}

		// Find the maximum index for each connectivity column (connectivity_index)
		std::vector<int32_t> max_indices(this->conn.stride, -1);
		std::vector<int32_t> min_indices(this->conn.stride, std::numeric_limits<int32_t>::max());
		
		for (int32_t elem = 0; elem < n_elements; elem++) {
			for (int32_t conn_idx = 0; conn_idx < this->conn.stride; conn_idx++) {
				const int32_t idx = conn_data[elem * this->conn.stride + conn_idx];
				max_indices[conn_idx] = std::max(max_indices[conn_idx], idx);
				min_indices[conn_idx] = std::min(min_indices[conn_idx], idx);
			}
		}

		// Now check each connectivity-indexed symbol data map
		for (size_t map_idx = 0; map_idx < this->maps.size(); map_idx++) {
			const DataMap<const FLOAT>& map = this->maps[map_idx];

			// Skip fixed (non-connectivity) maps
			if (map.connectivity_index < 0) {
				continue;
			}

			// Get symbol name from first symbol
			std::string symbol_name = "<unknown>";
			if (map.first_symbol_idx >= 0 && map.first_symbol_idx < n_symbols) {
				symbol_name = this->ws.get_expression_graph().get_symbol_names()[map.first_symbol_idx];
			}

			// Get the maximum index that will be accessed for this connectivity_index
			const int32_t max_idx = max_indices[map.connectivity_index];
			const int32_t min_idx = min_indices[map.connectivity_index];
			
			// Get the number of elements available in the data array
			const int32_t n_available = map.n_elements();

			// Check if we have enough data
			if (min_idx < 0 || max_idx >= n_available) {
				std::cout << "symx error: MappedWorkspace " << name_prefix << "symbol \"" << symbol_name 
						  << "\" with range [" << map.first_symbol_idx << "+" << map.stride << "]/" << n_symbols 
						  << " connectivity_index=" << map.connectivity_index << " out of bounds: min_index=" << min_idx
						  << " max_index=" << max_idx << " for valid range = [0, " << n_available << ")" << std::endl;
				exit(-1);
			}
		}
    }
    template <typename FLOAT>
    inline std::vector<Scalar> MappedWorkspace<FLOAT>::get_symbols(const DataMap<double> &data_map)
    {
        
        std::vector<Scalar> symbols;
		for (const auto& map : this->maps) {
			if (map.id() == data_map.id()) {  // Same container address
				for (int i = 0; i < map.stride; ++i) {
					symbols.push_back(this->ws.get_scalar(map.first_symbol_idx + i));
				}
			}
		}
        return symbols;
    }

    template <typename FLOAT>
    inline std::vector<DataMap<const FLOAT>> MappedWorkspace<FLOAT>::get_original_maps(const DataMap<double> &data_map)
    {
        // Find all DataMaps bound to the same data container by comparing container addresses
        std::vector<DataMap<const FLOAT>> original_maps;
		for (const auto& map : this->maps) {
			if (map.id() == data_map.id()) {  // Same container address
				original_maps.push_back(map);
			}
		}
        return original_maps;
    }

    template <typename FLOAT>
    inline void symx::MappedWorkspace<FLOAT>::_check_Eigen_stride_inferrable(const EigenMatrixRM<FLOAT> &arr, const std::string &operation_description) const
    {
		{
			if (arr.size() == 0) {
				const std::string name_prefix = this->name.empty() ? "" : "\"" + this->name + "\" ";
				std::cout << "symx error: MappedWorkspace " << name_prefix
						  << "cannot infer stride from uninitialized/empty Eigen matrix " << operation_description << std::endl;
				exit(-1);
			}
		}
    }

    template <typename FLOAT>
    using spMWS = std::shared_ptr<MappedWorkspace<FLOAT>>;
} // namespace symx
