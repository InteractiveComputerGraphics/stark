#include "DeformableVolumeHandler.h"

using namespace stark::models;

stark::models::DeformableVolumeHandler::DeformableVolumeHandler(const Id& id, std::shared_ptr<DeformableSolidsVolumes> volumes)
	: volumes(volumes), DeformableHandler(id, volumes->dyn, volumes->inertia, volumes->prescribed_positions)
{
}

DeformableVolumeHandler& stark::models::DeformableVolumeHandler::set_density(const double density)
{
	this->volumes->inertia->set_density(this->get_id(), density);
	return *this;
}
DeformableVolumeHandler& stark::models::DeformableVolumeHandler::set_inertia_damping(const double inertia_damping)
{
	this->volumes->inertia->set_inertia_damping(this->get_id(), inertia_damping);
	return *this;
}
DeformableVolumeHandler& stark::models::DeformableVolumeHandler::set_young_modulus(const double young_modulus)
{
	this->volumes->strain->set_young_modulus(this->get_id(), young_modulus);
	return *this;
}
DeformableVolumeHandler& stark::models::DeformableVolumeHandler::set_poisson_ratio(const double poisson_ratio)
{
	this->volumes->strain->set_poisson_ratio(this->get_id(), poisson_ratio);
	return *this;
}
DeformableVolumeHandler& stark::models::DeformableVolumeHandler::set_strain_damping(const double strain_damping)
{
	this->volumes->strain->set_strain_damping(this->get_id(), strain_damping);
	return *this;
}
DeformableVolumeHandler& stark::models::DeformableVolumeHandler::set_strain_limit(const double strain_limit)
{
	this->volumes->strain->set_strain_limit(this->get_id(), strain_limit);
	return *this;
}
DeformableVolumeHandler& stark::models::DeformableVolumeHandler::set_strain_limit_stiffness(const double strain_limit_stiffness)
{
	this->volumes->strain->set_strain_limit_stiffness(this->get_id(), strain_limit_stiffness);
	return *this;
}
DeformableVolumeHandler& stark::models::DeformableVolumeHandler::add_to_output_label(const std::string label)
{
	this->volumes->output_groups.add_to_group(label, this->get_volume_index());
	return *this;
}
double stark::models::DeformableVolumeHandler::get_density() const
{
	return this->volumes->inertia->get_density(this->get_id());
}
double stark::models::DeformableVolumeHandler::get_inertia_damping() const
{
	return this->volumes->inertia->get_inertia_damping(this->get_id());
}
double stark::models::DeformableVolumeHandler::get_young_modulus() const
{
	return this->volumes->strain->get_young_modulus(this->get_id());
}
double stark::models::DeformableVolumeHandler::get_poisson_ratio() const
{
	return this->volumes->strain->get_poisson_ratio(this->get_id());
}
double stark::models::DeformableVolumeHandler::get_strain_damping() const
{
	return this->volumes->strain->get_strain_damping(this->get_id());
}
double stark::models::DeformableVolumeHandler::get_strain_limit() const
{
	return this->volumes->strain->get_strain_limit(this->get_id());
}
double stark::models::DeformableVolumeHandler::get_strain_limit_stiffness() const
{
	return this->volumes->strain->get_strain_limit_stiffness(this->get_id());
}
int stark::models::DeformableVolumeHandler::get_volume_index() const
{
	return this->volumes->get_index(this->get_id());
}
