#include "../nanobind_stark_include_all.h"

void pystark_Settings(nb::module_& m)
{
    // symx enums
    nb::enum_<symx::LinearSolver>(m, "LinearSolver")
        .value("DirectLLT", symx::LinearSolver::DirectLLT)
        .value("BDPCG", symx::LinearSolver::BDPCG);

    nb::enum_<symx::ProjectionToPD>(m, "ProjectionToPD")
        .value("Newton", symx::ProjectionToPD::Newton)
        .value("ProjectedNewton", symx::ProjectionToPD::ProjectedNewton)
        .value("ProjectOnDemand", symx::ProjectionToPD::ProjectOnDemand)
        .value("Progressive", symx::ProjectionToPD::Progressive);

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
        .def_rw("file_verbosity",    &Settings::Output::file_verbosity)
        .def_rw("enable_frame_writes", &Settings::Output::enable_frame_writes)
        .def_rw("enable_output",     &Settings::Output::enable_output);

    auto simulation_struct = nb::class_<Settings::Simulation>(settings_struct, "Simulation")
        .def(nb::init<>())
        .def_rw("gravity", &Settings::Simulation::gravity)
        .def_rw("init_frictional_contact", &Settings::Simulation::init_frictional_contact)
        .def_rw("max_time_step_size", &Settings::Simulation::max_time_step_size)
        .def_rw("use_adaptive_time_step", &Settings::Simulation::use_adaptive_time_step)
        .def_rw("time_step_size_success_multiplier", &Settings::Simulation::time_step_size_success_multiplier)
        .def_rw("time_step_size_lower_bound", &Settings::Simulation::time_step_size_lower_bound);

    // symx::NewtonSettings bindings (inherits SolverSettings fields)
    auto newton_settings_struct = nb::class_<symx::NewtonSettings>(m, "NewtonSettings")
        .def(nb::init<>())
        // SolverSettings base fields
        .def_rw("max_iterations",                         &symx::NewtonSettings::max_iterations)
        .def_rw("min_iterations",                         &symx::NewtonSettings::min_iterations)
        .def_rw("residual_tolerance_abs",                 &symx::NewtonSettings::residual_tolerance_abs)
        .def_rw("residual_tolerance_rel",                 &symx::NewtonSettings::residual_tolerance_rel)
        .def_rw("step_tolerance",                         &symx::NewtonSettings::step_tolerance)
        .def_rw("max_iterations_as_success",              &symx::NewtonSettings::max_iterations_as_success)
        .def_rw("enable_armijo_backtracking",             &symx::NewtonSettings::enable_armijo_backtracking)
        .def_rw("max_backtracking_armijo_iterations",     &symx::NewtonSettings::max_backtracking_armijo_iterations)
        .def_rw("max_backtracking_invalid_state_iterations", &symx::NewtonSettings::max_backtracking_invalid_state_iterations)
        // NewtonSettings fields
        .def_rw("projection_mode",                        &symx::NewtonSettings::projection_mode)
        .def_rw("projection_eps",                         &symx::NewtonSettings::projection_eps)
        .def_rw("linear_solver",                          &symx::NewtonSettings::linear_solver)
        .def_rw("cg_max_iterations",                      &symx::NewtonSettings::cg_max_iterations)
        .def_rw("cg_abs_tolerance",                       &symx::NewtonSettings::cg_abs_tolerance)
        .def_rw("cg_rel_tolerance",                       &symx::NewtonSettings::cg_rel_tolerance)
        .def_rw("cg_stop_on_indefiniteness",              &symx::NewtonSettings::cg_stop_on_indefiniteness)
        .def_rw("bailout_residual",                       &symx::NewtonSettings::bailout_residual);

    auto execution_struct = nb::class_<Settings::Execution>(settings_struct, "Execution")
        .def(nb::init<>())
        .def_rw("allowed_execution_time", &Settings::Execution::allowed_execution_time)
        .def_rw("end_simulation_time", &Settings::Execution::end_simulation_time)
        .def_rw("end_frame", &Settings::Execution::end_frame)
        .def_rw("n_threads", &Settings::Execution::n_threads);
}
