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
using VecX = nb::ndarray<nb::numpy, T, nb::shape<-1>, nb::device::cpu>;

template<typename T, ssize_t M>
using MatX = nb::ndarray<nb::numpy, T, nb::shape<-1, M>, nb::device::cpu>;

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

template<typename T, ssize_t M>
inline std::vector<std::array<T, (size_t)M>> nb_to_stark(const MatX<T, M>& x)
{
    std::vector<std::array<T, (size_t)M>> arr(x.shape(0));
    for (size_t i = 0; i < arr.size(); ++i) {
        for (size_t j = 0; j < (size_t)M; j++) {
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
// We copy into heap memory and attach a nanobind capsule as owner.
// The capsule keeps the buffer alive until Python releases the array.
template<typename T>
inline VecX<T> stark_to_nb(const std::vector<T>& x)
{
    T* data = new T[x.size()];
    std::memcpy(data, x.data(), x.size() * sizeof(T));

    nb::capsule owner(data, [](void* p) noexcept {
        delete[] static_cast<T*>(p);
    });

    return VecX<T>(data, { x.size() }, owner);
}

inline MatX<double, 3> stark_to_nb(const std::vector<Eigen::Vector3d>& x)
{
    const size_t size = 3 * x.size();

    double* data = new double[size];
    std::memcpy(data, x.data(), size * sizeof(double));

    nb::capsule owner(data, [](void* p) noexcept {
        delete[] static_cast<double*>(p);
    });

    return MatX<double, 3>(data, { x.size(), 3 }, owner);
}

template<size_t M>
inline MatX<int, (ssize_t)M> stark_to_nb(const std::vector<std::array<int, M>>& x)
{
    const size_t size = M * x.size();

    int* data = new int[size];
    std::memcpy(data, x.data(), size * sizeof(int));

    nb::capsule owner(data, [](void* p) noexcept {
        delete[] static_cast<int*>(p);
    });

    return MatX<int, (ssize_t)M>(data, { x.size(), M }, owner);
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
