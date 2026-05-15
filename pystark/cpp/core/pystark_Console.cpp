#include "../nanobind_stark_include_all.h"

void pystark_Console(nb::module_& m)
{
    nb::enum_<symx::Verbosity>(m, "Verbosity")
        .value("Minimal",  symx::Verbosity::Minimal)
        .value("Summary",  symx::Verbosity::Summary)
        .value("Medium",   symx::Verbosity::Medium)
        .value("Full",     symx::Verbosity::Full);
}