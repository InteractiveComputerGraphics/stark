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
	 * Puts together a symbolic workspace with actual data bindings in the context of loop evaluation within SymX.
	 * However, this does not evaluate, it will be passed to one or multiple CompiledInLoop instances for evaluation of expressions.
	*/
    template<typename FLOAT>
    class MappedWorkspace
    {
    public:
        /* Fields */
        Workspace ws;
        Summation<FLOAT> summation;
        std::vector<DataMap<const FLOAT>> maps;
		ConnectivityMap conn;
		std::string name = "NO_NAME";

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
		inline Scalar make_scalar(std::function<const FLOAT* ()> data, const std::string& name = "");
		inline Scalar make_scalar(const FLOAT& scalar, const std::string& name = "");
		inline Vector make_vector(std::function<const FLOAT* ()> data, const int32_t stride, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline Vector make_vector(const STATIC_VECTOR& arr, const std::string& name = "");
		inline Matrix make_matrix(std::function<const FLOAT* ()> data, const std::array<int, 2> shape, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline Matrix make_matrix(const DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const std::string& name = "");

		// Make Scalar
		inline Scalar make_scalar(std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const Index& idx, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline Scalar make_scalar(const DYNAMIC_VECTOR& arr, const Index& idx, const std::string& name = "");

		// Make Scalars
		inline std::vector<Scalar> make_scalars(std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const std::vector<Index>& indices, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline std::vector<Scalar> make_scalars(const DYNAMIC_VECTOR& arr, const std::vector<Index>& indices, const std::string& name = "");

		// Make Vector
		inline Vector make_vector(std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const int32_t stride, const Index& idx, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline Vector make_vector(const std::vector<STATIC_VECTOR>& arr, const Index& idx, const std::string& name = "");
		inline Vector make_vector(const EigenMatrixRM<FLOAT>& arr, const Index& idx, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline Vector make_vector(const DYNAMIC_VECTOR& arr, const int32_t stride, const Index& idx, const std::string& name = "");

		// Make Vectors
		inline std::vector<Vector> make_vectors(std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const int32_t stride, const std::vector<Index>& indices, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline std::vector<Vector> make_vectors(const std::vector<STATIC_VECTOR>& arr, const std::vector<Index>& indices, const std::string& name = "");
		inline std::vector<Vector> make_vectors(const EigenMatrixRM<FLOAT>& arr, const std::vector<Index>& indices, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline std::vector<Vector> make_vectors(const std::vector<STATIC_VECTOR>& arr, const Element& element, const std::string& name = "");
		inline std::vector<Vector> make_vectors(const EigenMatrixRM<FLOAT>& arr, const Element& element, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline std::vector<Vector> make_vectors(const DYNAMIC_VECTOR& arr, const int32_t stride, const std::vector<Index>& indices, const std::string& name = "");

		// Make Matrix
		inline Matrix make_matrix(std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const std::array<int, 2> shape, const Index& idx, const std::string& name = "");
		template<typename STATIC_VECTOR>
		inline Matrix make_matrix(const std::vector<STATIC_VECTOR>& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name = "");
		inline Matrix make_matrix(const EigenMatrixRM<FLOAT>& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name = "");
		template<typename DYNAMIC_VECTOR>
		inline Matrix make_matrix(const DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name = "");

		// Make Matrices
		inline std::vector<Matrix> make_matrices(std::function<const FLOAT* ()> data, std::function<int32_t()> n_elements, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name = "");
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
		inline int get_n_elements() const;
		inline int get_connectivity_stride() const;
		inline const int32_t* get_connectivity_data() const;
		inline View<const int32_t> get_connectivity_elem(const int32_t i) const;
		inline const Workspace& get_workspace() const;

		// Verification
		inline void verify_data_maps() const;
		inline void verify_no_out_of_bounds() const;

		// Misc
		std::vector<Scalar> get_symbols(const DataMap<double>& data_map);
		std::vector<DataMap<const FLOAT>> get_original_maps(const DataMap<double>& data_map);
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
    // ============================================================================
	template<typename FLOAT>
	inline Scalar MappedWorkspace<FLOAT>::make_scalar(std::function<const FLOAT* ()> data, const std::string& name)
	{
		this->maps.emplace_back(
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
		return this->make_scalar([&scalar]() { return &scalar; }, name);
	}
	template<typename FLOAT>
	inline Vector MappedWorkspace<FLOAT>::make_vector(std::function<const FLOAT*()> data, const int32_t stride, const std::string& name)
	{
		this->maps.emplace_back(
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
		return this->make_vector([&arr]() { return arr.data(); }, (int32_t)arr.size(), name);
	}
	template<typename FLOAT>
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(std::function<const FLOAT*()> data, const std::array<int, 2> shape, const std::string& name)
	{
		this->maps.emplace_back(
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
		return this->make_matrix(std::function<const FLOAT*()>([&arr]() { return arr.data(); }), shape, name);
	}

	// ============================================================================
	// Make Scalar(s)
	// ============================================================================
	template<typename FLOAT>
	inline Scalar MappedWorkspace<FLOAT>::make_scalar(std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const Index& idx, const std::string& name)
	{
		this->maps.emplace_back(data, n_elements, 1, idx.idx, this->ws.get_n_symbols());
		return this->ws.make_scalar(name);
	}
	template<typename FLOAT>
	template<typename DYNAMIC_VECTOR>
	inline Scalar MappedWorkspace<FLOAT>::make_scalar(const DYNAMIC_VECTOR& arr, const Index& idx, const std::string& name)
	{
		return this->make_scalar(
			[&arr]() { return arr.data(); }, 
			[&arr]() { return (int32_t)arr.size(); }, 
			idx, 
			name);
	}
    template <typename FLOAT>
    inline std::vector<Scalar> MappedWorkspace<FLOAT>::make_scalars(std::function<const FLOAT *()> data, std::function<int32_t()> n_elements, const std::vector<Index> &indices, const std::string& name)
    {
        std::vector<Scalar> scalars;
        for (const auto &idx : indices) {
            scalars.push_back(this->make_scalar(data, n_elements, idx, name));
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
	// Make Vector(s)
	// ============================================================================
	template<typename FLOAT>
	inline Vector MappedWorkspace<FLOAT>::make_vector(std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const int32_t stride, const Index& idx, const std::string& name)
	{
		this->maps.emplace_back(data, n_elements, stride, idx.idx, this->ws.get_n_symbols());
		return this->ws.make_vector(name, stride);
	}
    template <typename FLOAT>
    inline Vector MappedWorkspace<FLOAT>::make_vector(const EigenMatrixRM<FLOAT> &arr, const Index &idx, const std::string& name)
    {
		return this->make_vector(
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
		return this->make_vector(
			[&arr]() { return arr[0].data(); },
			[&arr]() { return (int32_t)arr.size(); },
			(int32_t)arr[0].size(),
			idx,
			name);
	}
	template<typename FLOAT>
	template<typename DYNAMIC_VECTOR>
	inline Vector MappedWorkspace<FLOAT>::make_vector(const DYNAMIC_VECTOR& arr, const int32_t stride, const Index& idx, const std::string& name)
	{
		return this->make_vector(
			[&arr]() { return arr.data(); },
			[&arr, stride]() { return (int32_t)arr.size()/stride; },
			stride,
			idx,
			name);
	}

	template<typename FLOAT>
	inline std::vector<Vector> MappedWorkspace<FLOAT>::make_vectors(std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const int32_t stride, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Vector> vectors;
		for (size_t i = 0; i < indices.size(); i++) {
			vectors.push_back(this->make_vector(data, n_elements, stride, indices[i], name + std::to_string(i)));
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
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const std::array<int, 2> shape, const Index& idx, const std::string& name)
	{
		this->maps.emplace_back(data, n_elements, shape[0] * shape[1], idx.idx, this->ws.get_n_symbols());
		return this->ws.make_matrix(name, shape);
	}
	template<typename FLOAT>
	template<typename STATIC_VECTOR>
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(const std::vector<STATIC_VECTOR>& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name)
	{
		return this->make_matrix(
			[&arr]() { return arr[0].data(); },
			[&arr]() { return (int32_t)arr.size(); },
			shape,
			idx,
			name);
	}
	template<typename FLOAT>
	inline Matrix MappedWorkspace<FLOAT>::make_matrix(const EigenMatrixRM<FLOAT>& arr, const std::array<int, 2> shape, const Index& idx, const std::string& name)
	{
		return this->make_matrix(
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
			[&arr]() { return arr.data(); },
			[&arr, shape]() { return (int32_t)arr.size() / (shape[0] * shape[1]); },
			shape,
			idx,
			name);
	}

	template<typename FLOAT>
	inline std::vector<Matrix> MappedWorkspace<FLOAT>::make_matrices(std::function<const FLOAT*()> data, std::function<int32_t()> n_elements, const std::array<int, 2> shape, const std::vector<Index>& indices, const std::string& name)
	{
		std::vector<Matrix> matrices;
		for (size_t i = 0; i < indices.size(); i++) {
			matrices.push_back(this->make_matrix(data, n_elements, shape, indices[i], name + std::to_string(i)));
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
		const int32_t stride = (int32_t)iteration_vectors[0].size();
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
	inline int MappedWorkspace<FLOAT>::get_n_elements() const
	{
		return this->conn.n_elements();
	}

	template<typename FLOAT>
	inline int MappedWorkspace<FLOAT>::get_connectivity_stride() const
	{
		return this->conn.stride;
	}

	template<typename FLOAT>
	inline const int32_t* MappedWorkspace<FLOAT>::get_connectivity_data() const
	{
		return this->conn.data();
	}

	template<typename FLOAT>
	inline View<const int32_t> MappedWorkspace<FLOAT>::get_connectivity_elem(const int32_t i) const
	{
		return View<const int32_t>(this->conn.data() + this->conn.stride*i, this->conn.stride);
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
			if ((const void*)map.data() == (const void*)data_map.data()) {  // Compare data pointers
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
        std::vector<DataMap<const FLOAT>> original_maps;
		for (const auto& map : this->maps) {
			if ((const void*)map.data() == (const void*)data_map.data()) {  // Compare data pointers
				original_maps.push_back(map);
			}
		}
        return original_maps;
    }

    template <typename FLOAT>
    using spMWS = std::shared_ptr<MappedWorkspace<FLOAT>>;
} // namespace symx
