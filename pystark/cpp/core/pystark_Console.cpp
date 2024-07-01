#include "../nanobind_stark_include_all.h"

using namespace stark::core;

void pystark_Console(nb::module_& m)
{
    // Enums
    nb::enum_<stark::ConsoleOutputTo>(m, "ConsoleOutputTo")
        .value("ConsoleOnly", stark::ConsoleOutputTo::ConsoleOnly)
        .value("FileOnly", stark::ConsoleOutputTo::FileOnly)
        .value("FileAndConsole", stark::ConsoleOutputTo::FileAndConsole)
        .value("NoOutput", stark::ConsoleOutputTo::NoOutput);

    nb::enum_<stark::ConsoleVerbosity>(m, "ConsoleVerbosity")
        .value("NoOutput", stark::ConsoleVerbosity::NoOutput)
		.value("Frames", stark::ConsoleVerbosity::Frames)
		.value("TimeSteps", stark::ConsoleVerbosity::TimeSteps)
		.value("NewtonIterations", stark::ConsoleVerbosity::NewtonIterations);

    // Class
    nb::class_<core::Console>(m, "Console")
		.def(nb::init<>())
		.def("initialize", &Console::initialize)
		.def("set_path", &Console::set_path)
		.def("get_frame_path", &Console::get_frame_path)
		.def("set_verbosity", &Console::set_verbosity)
		.def("set_output_target", &Console::set_output_target)
		.def("print", &Console::print)
		.def("add_error_msg", &Console::add_error_msg)
		.def("print_error_msg_and_clear", &Console::print_error_msg_and_clear)
		;
}