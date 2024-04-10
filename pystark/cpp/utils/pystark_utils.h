#include <nanobind/nanobind.h>
namespace nb = nanobind;

void pystark_utils_impl(nb::module_& m);

void pystark_utils(nb::module_& m)
{
	pystark_utils_impl(m);
}
