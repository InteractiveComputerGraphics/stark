#pragma once
#include <nanobind/nanobind.h>
namespace nb = nanobind;
using namespace nb::literals;

#include <nanobind/ndarray.h>
#include <nanobind/stl/unordered_set.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/eigen/dense.h>

#include <cstring>


// Type shortcuts
#include <stark>
using namespace stark;
using PH = const PointSetHandler;
using RBH = const RigidBodyHandler;

// nanobind <-> numpy types
template<typename T>
using VecX = nb::ndarray<nb::numpy, T, nb::shape<nb::any>>;
template<typename T, std::size_t M>
using MatX = nb::ndarray<nb::numpy, T, nb::shape<nb::any, M>>;

using VecXd = VecX<double>;
using MatX2d = MatX<double, 2>;
using MatX3d = MatX<double, 3>;
using VecXi = VecX<int>;
using MatX2i = MatX<int, 2>;
using MatX3i = MatX<int, 3>;
using MatX4i = MatX<int, 4>;


// nanobind -> stark converters
template<typename T>
inline std::vector<T> nb_to_stark(const VecX<T>& x)
{
	std::vector<T> arr(x.shape(0));
	for (size_t i = 0; i < arr.size(); ++i) {
		arr[i] = x(i);
	}
	return arr;
}
inline std::vector<Eigen::Vector3d> nb_to_stark(const MatX3d& x) 
{
	std::vector<Eigen::Vector3d> arr(x.shape(0));
	for (size_t i = 0; i < arr.size(); ++i) {
		for (size_t j = 0; j < 3; j++) {
			arr[i][j] = x(i, j);
		}
	}
	return arr;
}
template<typename T, std::size_t M>
inline std::vector<std::array<T, M>> nb_to_stark(const MatX<T, M>& x)
{
	std::vector<std::array<T, M>> arr(x.shape(0));
	for (size_t i = 0; i < arr.size(); ++i) {
		for (size_t j = 0; j < M; j++) {
			arr[i][j] = x(i, j);
		}
	}
	return arr;
}
inline std::vector<std::array<double, 3>> nb_to_stark_no_eigen(const MatX<double, 3>& x)
{
	std::vector<std::array<double, 3>> arr(x.shape(0));
	for (size_t i = 0; i < arr.size(); ++i) {
		for (size_t j = 0; j < 3; j++) {
			arr[i][j] = x(i, j);
		}
	}
	return arr;
}

// stark -> nanobind converters
// Note: In order to pass numpy arrays by copied data to Python, we need to manage their lifetime in C++
//       We use a nanobind::capsule to do this
template<typename T>
inline VecX<T> stark_to_nb(const std::vector<T>& x) {

	// Copy data to a dynamically allocated array that will be destroyed when the capsule is destroyed
	T* data = new T[x.size()];
	memcpy(data, x.data(), x.size() * sizeof(T));

	// Deferred destructor
	nb::capsule owner(data, [](void* p) noexcept { delete[](T*) p; });

	const std::array<size_t, 1> shape = { x.size() };
	return VecX<T>(data, 1, shape.data(), owner);
}
inline MatX<double, 3> stark_to_nb(const std::vector<Eigen::Vector3d>& x) {

	// Copy data to a dynamically allocated array that will be destroyed when the capsule is destroyed
	const size_t size = 3*x.size();
	double* data = new double[size];
	memcpy(data, x.data(), size * sizeof(double));

	// Deferred destructor
	nb::capsule owner(data, [](void* p) noexcept { delete[](double*) p; });

	const std::array<size_t, 2> shape = { x.size(), 3 };
	return MatX<double, 3>(data, 2, shape.data(), owner);
}
template<std::size_t M>
inline MatX<int, M> stark_to_nb(const std::vector<std::array<int, M>>& x) {

	// Copy data to a dynamically allocated array that will be destroyed when the capsule is destroyed
	const size_t size = M * x.size();
	int* data = new int[size];
	memcpy(data, x.data(), size * sizeof(int));

	// Deferred destructor
	nb::capsule owner(data, [](void* p) noexcept { delete[](int*) p; });

	const std::array<size_t, 2> shape = { x.size(), M };
	return MatX<int, M>(data, 2, shape.data(), owner);
}


// Stark parameter and handler macros for bindings
#define BIND_PARAMS(ParamsClass) \
	nb::class_<ParamsClass>(model, #ParamsClass) \
        .def(nb::init<>()) \

#define PARAM(ParamName) \
	.def_rw(#ParamName, &Params::ParamName) \
	.def("set_" #ParamName, &Params::set_##ParamName) \
	.def("get_" #ParamName, &Params::get_##ParamName) \

#define PARAM_(ParamsClass, ParamName) \
	.def_rw(#ParamName, &ParamsClass::ParamName) \
	.def("set_" #ParamName, &ParamsClass::set_##ParamName) \
	.def("get_" #ParamName, &ParamsClass::get_##ParamName) \

#define PARAM_STRAIN_LIMITING() PARAM(strain_limit) PARAM(strain_limit_stiffness)

#define BIND_HANDLER(HandlerClass) \
	nb::class_<HandlerClass>(model, #HandlerClass) \
	    .def("get_idx", &Handler::get_idx) \
	    .def("get_params", &Handler::get_params) \
	    .def("set_params", &Handler::set_params) \
	    .def("is_valid", &Handler::is_valid) \
	    .def("exit_if_not_valid", &Handler::exit_if_not_valid) \

// Utility function to create a named tuple in C++
inline nb::object create_named_tuple(const std::string& typeName, const std::vector<std::string>& fieldNames, const nb::tuple& values) {
	nb::object collections = nb::module_::import_("collections");
	nb::object namedtuple = collections.attr("namedtuple");

	// Create a Python list for field names
	nb::list fields;
	for (const auto& fieldName : fieldNames) {
		fields.append(fieldName);
	}
	nb::object tupleType = namedtuple(typeName.c_str(), fields);
	return tupleType(*values);
}
