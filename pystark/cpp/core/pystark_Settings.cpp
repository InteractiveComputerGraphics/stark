#include "../nanobind_stark_include_all.h"

void pystark_Settings(nb::module_& m)
{
    // symx enums
    nb::enum_<symx::LinearSolver>(m, "LinearSolver")
        .value("DirectLU", symx::LinearSolver::DirectLU)
        .value("BDPCG", symx::LinearSolver::BDPCG);

    nb::enum_<symx::ProjectionToPD>(m, "ProjectionToPD")
        .value("Newton", symx::ProjectionToPD::Newton)
        .value("ProjectedNewton", symx::ProjectionToPD::ProjectedNewton)
        .value("ProjectOnDemand", symx::ProjectionToPD::ProjectOnDemand)
        .value("Progressive", symx::ProjectionToPD::Progressive);

    // Functions
    m.def("set_compiler_command", &set_compiler_command);

    // Structs
    auto settings_struct = nb::class_<Settings>(m, "Settings")
        .def(nb::init<>())
        .def("as_string", &Settings::as_string)
        .def_rw("output", &Settings::output)
        .def_rw("simulation", &Settings::simulation)
        .def_rw("newton", &Settings::newton)
        .def_rw("execution", &Settings::execution);

    auto output_struct = nb::class_<Settings::Output>(settings_struct, "Output")
        .def(nb::init<>())
        .def_rw("simulation_name", &Settings::Output::simulation_name)
        .def_rw("output_directory", &Settings::Output::output_directory)
        .def_rw("codegen_directory", &Settings::Output::codegen_directory)
        .def_rw("time_stamp", &Settings::Output::time_stamp)
        .def_rw("fps", &Settings::Output::fps)
        .def_rw("console_verbosity", &Settings::Output::console_verbosity)
        .def_rw("console_output_to", &Settings::Output::console_output_to)
        .def_rw("enable_output", &Settings::Output::enable_output);

    auto simulation_struct = nb::class_<Settings::Simulation>(settings_struct, "Simulation")
        .def(nb::init<>())
        .def_rw("gravity", &Settings::Simulation::gravity)
        .def_rw("init_frictional_contact", &Settings::Simulation::init_frictional_contact)
        .def_rw("max_time_step_size", &Settings::Simulation::max_time_step_size)
        .def_rw("use_adaptive_time_step", &Settings::Simulation::use_adaptive_time_step)
        .def_rw("time_step_size_success_muliplier", &Settings::Simulation::time_step_size_success_muliplier)
        .def_rw("time_step_size_lower_bound", &Settings::Simulation::time_step_size_lower_bound);

    // symx::NewtonSettings bindings
    auto newton_settings_struct = nb::class_<symx::NewtonSettings>(m, "NewtonSettings")
        .def(nb::init<>())
        .def_rw("max_iterations", &symx::NewtonSettings::max_iterations)
        .def_rw("max_line_search_iterations", &symx::NewtonSettings::max_line_search_iterations)
        .def_rw("residual_tolerance", &symx::NewtonSettings::residual_tolerance)
        .def_rw("linear_solver", &symx::NewtonSettings::linear_solver)
        .def_rw("projection_mode", &symx::NewtonSettings::projection_mode)
        .def_rw("projection_eps", &symx::NewtonSettings::projection_eps)
        .def_rw("cg_max_iterations", &symx::NewtonSettings::cg_max_iterations)
        .def_rw("cg_abs_tolerance", &symx::NewtonSettings::cg_abs_tolerance)
        .def_rw("cg_rel_tolerance", &symx::NewtonSettings::cg_rel_tolerance)
        .def_rw("bailout_residual", &symx::NewtonSettings::bailout_residual);

    auto execution_struct = nb::class_<Settings::Execution>(settings_struct, "Execution")
        .def(nb::init<>())
        .def_rw("allowed_execution_time", &Settings::Execution::allowed_execution_time)
        .def_rw("end_simulation_time", &Settings::Execution::end_simulation_time)
        .def_rw("end_frame", &Settings::Execution::end_frame)
        .def_rw("n_threads", &Settings::Execution::n_threads);
}
