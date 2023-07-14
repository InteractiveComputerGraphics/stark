#pragma once
#include <functional>
#include <vector>
#include <array>
#include <cstdint>
#include <iostream>

#include <Eigen/Dense>

namespace symx
{
	/*
		Types supported:
			double:
				- double
				- std::vector<double>
				- Eigen::VectorXd
				- std::vector<std::array<double, N>>
				- std::vector<Eigen::Matrix<double, N, 1>>
				- Eigen::Matrix<double, M, N>
			
			int:
				- std::vector<int32_t>
				- std::vector<std::array<int32_t, N>>
	*/

	// Lambda to the begin pointer
	// double
	//// double
	inline std::function<const double* ()> l2data_double(const double& v) { return [&v]() { return &v; }; }
	
	//// std::vector<double>, Eigen::VectorXd, Eigen::MatrixXd
	template<typename DYNAMIC_VECTOR>
	inline std::function<const double* ()> l2data_double(const DYNAMIC_VECTOR& v) { return [&v]() { return v.data(); }; }
	template<typename DYNAMIC_VECTOR>
	inline std::function<double* ()> l2data_double_mut(DYNAMIC_VECTOR& v) { return [&v]() { return v.data(); }; }

	// std::vector<std::array<double, N>>, std::vector<Eigen::Vector3d>
	template<typename STATIC_VECTOR>
	inline std::function<const double* ()> l2data_double(const std::vector<STATIC_VECTOR>& v) { return [&v]() { return &v[0][0]; }; }
	inline std::function<const double* ()> l2data_double(const std::vector<double>& v) { return [&v]() { return v.data(); }; } // Guard against template

	template<typename STATIC_VECTOR>
	inline std::function<double* ()> l2data_double_mut(std::vector<STATIC_VECTOR>& v) { return [&v]() { return &v[0][0]; }; }
	inline std::function<double* ()> l2data_double_mut(std::vector<double>& v) { return [&v]() { return v.data(); }; } // Guard against template


	inline std::function<int32_t()> l2count_double(const std::vector<double>& v) { return [&v]() { return (int32_t)v.size(); }; }
	template<typename DYNAMIC_VECTOR>
	inline std::function<int32_t()> l2count_double(const DYNAMIC_VECTOR& v) { return [&v]() { return (int32_t)v.size(); }; }
	template<typename STATIC_VECTOR>
	inline std::function<int32_t()> l2count_double(const std::vector<STATIC_VECTOR>& v) { return [&v]() { return (int32_t)(v.size()*sizeof(STATIC_VECTOR)/8); }; }


	// int
	inline std::function<const int32_t* ()> l2data_int(const std::vector<int32_t>& v) { return [&v]() { return v.data(); }; }
	template<std::size_t N>
	inline std::function<const int32_t* ()> l2data_int(const std::vector<std::array<int32_t, N>>& v) { return [&v]() { return &v[0][0]; }; }


	// Lambda to the outer size (number of points/elements/rows...)
	inline std::function<int32_t()> l2n_elements_int(const std::vector<int32_t>& v, const int32_t stride) { return [&v, stride]() { return (int32_t)v.size() / stride; }; };
	template<std::size_t N>
	inline std::function<int32_t()> l2n_elements_int(const std::vector<std::array<int32_t, N>>& v) { return [&v]() { return (int32_t)v.size(); }; };
}
