#include "DeformableSurfaceHandler.h"

using namespace stark::models;

stark::models::DeformableSurfaceHandler::DeformableSurfaceHandler(const Id& id, std::shared_ptr<DeformableSolidsSurfaces> surfaces)
	: surfaces(surfaces), DeformableHandler(id, surfaces->dyn, surfaces->inertia, surfaces->prescribed_positions)
{
}

DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_area_density(const double density)
{
	this->surfaces->inertia->set_density(this->get_id(), density);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_thickness(const double thickness)
{
	this->surfaces->strain->set_thickness(this->get_id(), thickness);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_inertia_damping(const double inertia_damping)
{
	this->surfaces->inertia->set_inertia_damping(this->get_id(), inertia_damping);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_young_modulus(const double young_modulus)
{
	this->surfaces->strain->set_young_modulus(this->get_id(), young_modulus);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_poisson_ratio(const double poisson_ratio)
{
	this->surfaces->strain->set_poisson_ratio(this->get_id(), poisson_ratio);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_strain_damping(const double strain_damping)
{
	this->surfaces->strain->set_strain_damping(this->get_id(), strain_damping);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_strain_limit(const double strain_limit)
{
	this->surfaces->strain->set_strain_limit(this->get_id(), strain_limit);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_strain_limit_stiffness(const double strain_limit_stiffness)
{
	this->surfaces->strain->set_strain_limit_stiffness(this->get_id(), strain_limit_stiffness);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_bending_stiffness(const double bending_stiffness)
{
	this->surfaces->bending_grispun_03->set_stiffness(this->get_id(), bending_stiffness);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::set_bending_damping(const double bending_damping)
{
	this->surfaces->bending_grispun_03->set_damping(this->get_id(), bending_damping);
	return *this;
}
DeformableSurfaceHandler& stark::models::DeformableSurfaceHandler::add_to_output_label(const std::string label)
{
	this->surfaces->output_groups.add_to_group(label, this->get_surface_index());
	return *this;
}
double stark::models::DeformableSurfaceHandler::get_area_density() const
{
	return this->surfaces->inertia->get_density(this->get_id());
}
double stark::models::DeformableSurfaceHandler::get_thickness() const
{
	return this->surfaces->strain->get_thickness(this->get_id());
}
double stark::models::DeformableSurfaceHandler::get_inertia_damping() const
{
	return this->surfaces->inertia->get_inertia_damping(this->get_id());
}
double stark::models::DeformableSurfaceHandler::get_young_modulus() const
{
	return this->surfaces->strain->get_young_modulus(this->get_id());
}
double stark::models::DeformableSurfaceHandler::get_poisson_ratio() const
{
	return this->surfaces->strain->get_poisson_ratio(this->get_id());
}
double stark::models::DeformableSurfaceHandler::get_strain_damping() const
{
	return this->surfaces->strain->get_strain_damping(this->get_id());
}
double stark::models::DeformableSurfaceHandler::get_strain_limit() const
{
	return this->surfaces->strain->get_strain_limit(this->get_id());
}
double stark::models::DeformableSurfaceHandler::get_strain_limit_stiffness() const
{
	return this->surfaces->strain->get_strain_limit_stiffness(this->get_id());
}
double stark::models::DeformableSurfaceHandler::get_bending_stiffness() const
{
	return this->surfaces->bending_grispun_03->get_stiffness(this->get_id());
}
double stark::models::DeformableSurfaceHandler::get_bending_damping() const
{
	return this->surfaces->bending_grispun_03->get_damping(this->get_id());
}
int stark::models::DeformableSurfaceHandler::get_surface_index() const
{
	return this->surfaces->get_index(this->get_id());
}
