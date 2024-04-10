#include "../nanobind_stark_include_all.h"

void pystark_Simulation(nb::module_& m)
{
    nb::class_<Simulation>(m, "Simulation")
        .def(nb::init<const core::Settings&>())
        .def("get_time", &Simulation::get_time)
        .def("get_time_step_size", &Simulation::get_time_step_size)
        .def("get_frame", &Simulation::get_frame)
        .def("get_gravity", &Simulation::get_gravity)
        .def("set_gravity", &Simulation::set_gravity)
        .def("logger", [](Simulation& self) { return &self.get_logger(); }, nb::rv_policy::reference_internal)
        .def("console", [](Simulation& self) { return &self.get_console(); }, nb::rv_policy::reference_internal)
        .def("settings", [](Simulation& self) { return &self.get_settings(); }, nb::rv_policy::reference_internal)
        .def("add_time_event", nb::overload_cast<double, double, std::function<void(double)>>(& Simulation::add_time_event))
        .def("run", nb::overload_cast<double, std::function<void()>>(&Simulation::run), "duration"_a, "callback"_a = std::function<void()>())
        .def("run", nb::overload_cast<std::function<void()>>(&Simulation::run), "callback"_a = std::function<void()>())
        .def("run_one_time_step", &Simulation::run_one_time_step)
        
        .def("deformables", [](Simulation& self) { return self.deformables.get(); }, nb::rv_policy::reference_internal)
        .def("rigidbodies", [](Simulation& self) { return self.rigidbodies.get(); }, nb::rv_policy::reference_internal)
        .def("interactions", [](Simulation& self) { return self.interactions.get(); }, nb::rv_policy::reference_internal)
        .def("presets", [](Simulation& self) { return self.presets.get(); }, nb::rv_policy::reference_internal)
        ;
}
