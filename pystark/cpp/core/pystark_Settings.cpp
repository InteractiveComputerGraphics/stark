#include "../nanobind_stark_include_all.h"

void pystark_Settings(nb::module_& m)
{
    // Enums
    nb::enum_<stark::ResidualType>(m, "ResidualType")
        .value("Force", stark::ResidualType::Force)
        .value("Acceleration", stark::ResidualType::Acceleration)
        .export_values();

    nb::enum_<stark::LinearSystemSolver>(m, "LinearSystemSolver")
        .value("CG", stark::LinearSystemSolver::CG)
        .value("DirectLU", stark::LinearSystemSolver::DirectLU);

    nb::class_<stark::Residual>(m, "Residual")
        .def(nb::init<>())
        .def_rw("type", &stark::Residual::type)
        .def_rw("tolerance", &stark::Residual::tolerance);

    // Functions
    m.def("set_compiler_command", &set_compiler_command);

    // Structs
    auto settings_struct = nb::class_<Settings>(m, "Settings")
        .def(nb::init<>())
        .def("as_string", &Settings::as_string)
        .def_rw("output", &Settings::output)
        .def_rw("simulation", &Settings::simulation)
        .def_rw("newton", &Settings::newton)
        .def_rw("execution", &Settings::execution)
        .def_rw("debug", &Settings::debug);

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

    auto newtons_method_struct = nb::class_<Settings::NewtonsMethod>(settings_struct, "NewtonsMethod")
        .def(nb::init<>())
        .def_rw("residual", &Settings::NewtonsMethod::residual)
        .def_rw("linear_system_solver", &Settings::NewtonsMethod::linear_system_solver)
        .def_rw("project_to_PD", &Settings::NewtonsMethod::project_to_PD)
        .def_rw("max_newton_iterations", &Settings::NewtonsMethod::max_newton_iterations)
        .def_rw("max_line_search_iterations", &Settings::NewtonsMethod::max_line_search_iterations)
        .def_rw("line_search_multiplier", &Settings::NewtonsMethod::line_search_multiplier)
        .def_rw("cg_max_iterations_multiplier", &Settings::NewtonsMethod::cg_max_iterations_multiplier);

    auto execution_struct = nb::class_<Settings::Execution>(settings_struct, "Execution")
        .def(nb::init<>())
        .def_rw("allowed_execution_time", &Settings::Execution::allowed_execution_time)
        .def_rw("end_simulation_time", &Settings::Execution::end_simulation_time)
        .def_rw("end_frame", &Settings::Execution::end_frame)
        .def_rw("n_threads", &Settings::Execution::n_threads);

    auto debug_struct = nb::class_<Settings::Debug>(settings_struct, "Debug")
        .def(nb::init<>())
        .def_rw("symx_check_for_NaNs", &Settings::Debug::symx_check_for_NaNs)
        .def_rw("symx_suppress_compiler_output", &Settings::Debug::symx_suppress_compiler_output)
        .def_rw("symx_force_compilation", &Settings::Debug::symx_force_compilation)
        .def_rw("symx_force_load", &Settings::Debug::symx_force_load)
        .def_rw("symx_finite_difference_check", &Settings::Debug::symx_finite_difference_check)
        .def_rw("line_search_output", &Settings::Debug::line_search_output);

}
