#include "../../nanobind_stark_include_all.h"

void pystark_deformables_preset_types(nb::module_& m)
{
	auto line_module = m.def_submodule("Line");
	{
		nb::class_<Line::Handler>(line_module, "Handler")
			.def_rw("point_set", &Line::Handler::point_set)
			.def_rw("inertia", &Line::Handler::inertia)
			.def_rw("strain", &Line::Handler::strain)
			.def_rw("contact", &Line::Handler::contact)
			;

		nb::enum_<Line::Preset>(line_module, "Preset")
			.value("Elastic_Rubberband", Line::Preset::Elastic_Rubberband)
			;

		nb::class_<Line::Params>(line_module, "Params")
			.def_rw("inertia", &Line::Params::inertia)
			.def_rw("strain", &Line::Params::strain)
			.def_rw("contact", &Line::Params::contact)

			.def(nb::init<>())
			.def(nb::init<Line::Preset>(), "preset"_a)
			.def_static("Elastic_Rubberband", &Line::Params::Elastic_Rubberband)
			;
	}

	auto surface_module = m.def_submodule("Surface");
	{
		nb::class_<Surface::Handler>(surface_module, "Handler")
			.def_rw("point_set", &Surface::Handler::point_set)
			.def_rw("inertia", &Surface::Handler::inertia)
			.def_rw("strain", &Surface::Handler::strain)
			.def_rw("bending", &Surface::Handler::bending)
			.def_rw("contact", &Surface::Handler::contact)
			;

		nb::enum_<Surface::Preset>(surface_module, "Preset")
			.value("Cotton_Fabric", Surface::Preset::Cotton_Fabric)
			;

		nb::class_<Surface::Params>(surface_module, "Params")
			.def_rw("inertia", &Surface::Params::inertia)
			.def_rw("strain", &Surface::Params::strain)
			.def_rw("bending", &Surface::Params::bending)
			.def_rw("contact", &Surface::Params::contact)

			.def(nb::init<>())
			.def(nb::init<Surface::Preset>(), "preset"_a)
			.def_static("Cotton_Fabric", &Surface::Params::Cotton_Fabric)
			;
	}
	auto prescribed_surface_module = m.def_submodule("PrescribedSurface");
	{
		nb::class_<PrescribedSurface::Handler>(prescribed_surface_module, "Handler")
			.def_rw("point_set", &PrescribedSurface::Handler::point_set)
			.def_rw("inertia", &PrescribedSurface::Handler::prescribed)
			.def_rw("contact", &PrescribedSurface::Handler::contact)
			;

		nb::class_<PrescribedSurface::Params>(prescribed_surface_module, "Params")
			.def_rw("inertia", &PrescribedSurface::Params::prescribed)
			.def_rw("contact", &PrescribedSurface::Params::contact)

			.def(nb::init<>())
			;
	}

	auto volume_module = m.def_submodule("Volume");
	{
		nb::class_<Volume::Handler>(volume_module, "Handler")
			.def_rw("point_set", &Volume::Handler::point_set)
			.def_rw("inertia", &Volume::Handler::inertia)
			.def_rw("strain", &Volume::Handler::strain)
			.def_rw("contact", &Volume::Handler::contact)
			;

		nb::enum_<Volume::Preset>(volume_module, "Preset")
			.value("Soft_Rubber", Volume::Preset::Soft_Rubber)
			;

		nb::class_<Volume::Params>(volume_module, "Params")
			.def_rw("inertia", &Volume::Params::inertia)
			.def_rw("strain", &Volume::Params::strain)
			.def_rw("contact", &Volume::Params::contact)

			.def(nb::init<>())
			.def(nb::init<Volume::Preset>(), "preset"_a)
			.def_static("Soft_Rubber", &Volume::Params::Soft_Rubber)
			;
	}

}
