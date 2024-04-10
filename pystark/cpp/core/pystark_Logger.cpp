#include "../nanobind_stark_include_all.h"

using namespace stark::core;

void pystark_Logger(nb::module_& m)
{
    nb::class_<core::Logger>(m, "Logger")
        .def(nb::init<>())
        .def("start_timing", &Logger::start_timing)
        .def("stop_timing_series", &Logger::stop_timing_series)
        .def("stop_timing_add", &Logger::stop_timing_add)
        .def("add_to_timer", &Logger::add_to_timer)
        .def("add_to_counter", &Logger::add_to_counter)
        .def("save_to_disk", nb::overload_cast<const std::string&>(&Logger::save_to_disk))
        .def("save_to_disk", nb::overload_cast<>(&Logger::save_to_disk))
        .def("set_path", &Logger::set_path)
        .def("set", nb::overload_cast<const std::string&, const double>(&Logger::set))
        .def("set", nb::overload_cast<const std::string&, const int>(&Logger::set))
        .def("add", nb::overload_cast<const std::string&, const double>(&Logger::add))
        .def("add", nb::overload_cast<const std::string&, const int>(&Logger::add))
        .def("append_to_series", nb::overload_cast<const std::string&, const std::string&>(&Logger::append_to_series))
        .def("append_to_series", nb::overload_cast<const std::string&, const double>(&Logger::append_to_series))
        .def("append_to_series", nb::overload_cast<const std::string&, const int>(&Logger::append_to_series))
        .def("get_series", &Logger::get_series)
        .def("get_double", &Logger::get_double)
        .def("get_int", &Logger::get_int)
        .def("get_path", &Logger::get_path);
}