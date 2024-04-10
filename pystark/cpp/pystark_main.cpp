#include "nanobind_stark_include_all.h"

#include "core/pystark_core.h"
#include "models/pystark_models.h"
#include "utils/pystark_utils.h"

NB_MODULE(pystark, m) {
	pystark_core(m);
	pystark_models(m);
	pystark_utils(m);
}
