#include "../nanobind_stark_include_all.h"

void pystark_Logger(nb::module_& m);
void pystark_Console(nb::module_& m);
void pystark_Settings(nb::module_& m);

void pystark_core(nb::module_& m)
{
	pystark_Logger(m);
	pystark_Console(m);
	pystark_Settings(m);
}