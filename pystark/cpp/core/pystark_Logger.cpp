#include "../nanobind_stark_include_all.h"

void pystark_Logger(nb::module_& m)
{
    nb::class_<symx::Logger::Stats>(m, "LoggerStats")
        .def_ro("total", &symx::Logger::Stats::total)
        .def_ro("avg",   &symx::Logger::Stats::avg)
        .def_ro("min",   &symx::Logger::Stats::min)
        .def_ro("max",   &symx::Logger::Stats::max)
        .def_ro("found", &symx::Logger::Stats::found);

    nb::class_<symx::Logger>(m, "Logger")
        // Timing
        .def("start_timing",  &symx::Logger::start_timing)
        .def("stop_timing",   &symx::Logger::stop_timing)

        // Series
        .def("append", nb::overload_cast<const std::string&, double>(&symx::Logger::append))
        .def("append", nb::overload_cast<const std::string&, int>(&symx::Logger::append))
        .def("append", nb::overload_cast<const std::string&, const std::string&>(&symx::Logger::append))

        // Accumulators
        .def("set", nb::overload_cast<const std::string&, double>(&symx::Logger::set))
        .def("set", nb::overload_cast<const std::string&, int>(&symx::Logger::set))
        .def("add", nb::overload_cast<const std::string&, double>(&symx::Logger::add))
        .def("add", nb::overload_cast<const std::string&, int>(&symx::Logger::add))

        // Add and append
        .def("add_and_append", nb::overload_cast<const std::string&, double>(&symx::Logger::add_and_append))
        .def("add_and_append", nb::overload_cast<const std::string&, int>(&symx::Logger::add_and_append))

        // Getters — scalars
        .def("get_double", &symx::Logger::get_double)
        .def("get_int",    &symx::Logger::get_int)

        // Getters — series (copied to Python lists)
        .def("get_double_series", [](const symx::Logger& self, const std::string& label) {
            return std::vector<double>(self.get_double_series(label)); })
        .def("get_int_series", [](const symx::Logger& self, const std::string& label) {
            return std::vector<int>(self.get_int_series(label)); })
        .def("get_string_series", [](const symx::Logger& self, const std::string& label) {
            return std::vector<std::string>(self.get_string_series(label)); })

        // Timer queries
        .def("get_timer_total",  &symx::Logger::get_timer_total)
        .def("get_timer_count",  &symx::Logger::get_timer_count)
        .def("get_timer_avg",    &symx::Logger::get_timer_avg)
        .def("get_timer_min",    &symx::Logger::get_timer_min)
        .def("get_timer_max",    &symx::Logger::get_timer_max)
        .def("get_timer_labels", [](const symx::Logger& self) {
            return std::vector<std::string>(self.get_timer_labels()); })

        // Statistics
        .def("get_stats",           &symx::Logger::get_stats)
        .def("get_stats_as_string", &symx::Logger::get_stats_as_string)

        // Persistence
        .def("set_path",  &symx::Logger::set_path)
        .def("get_path",  &symx::Logger::get_path)
        .def("save_to_disk", nb::overload_cast<const std::string&>(&symx::Logger::save_to_disk))
        .def("save_to_disk", nb::overload_cast<>(&symx::Logger::save_to_disk))
        .def("time_since_last_write", &symx::Logger::time_since_last_write)

        // Control
        .def("clear", &symx::Logger::clear)
        ;
}