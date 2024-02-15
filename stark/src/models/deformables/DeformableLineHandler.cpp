#include "DeformableLineHandler.h"

using namespace stark::models;

stark::models::DeformableLineHandler::DeformableLineHandler(const Id& id, std::shared_ptr<DeformableSolidsLines> lines)
	: lines(lines), DeformableHandler(id, lines->dyn, lines->inertia, lines->prescribed_positions)
{
}

DeformableLineHandler& stark::models::DeformableLineHandler::set_linear_density(const double density)
{
	this->lines->inertia->set_density(this->get_id(), density);
	return *this;
}
DeformableLineHandler& stark::models::DeformableLineHandler::set_inertia_damping(const double inertia_damping)
{
	this->lines->inertia->set_inertia_damping(this->get_id(), inertia_damping);
	return *this;
}
DeformableLineHandler& stark::models::DeformableLineHandler::set_young_modulus(const double young_modulus)
{
	this->lines->strain->set_young_modulus(this->get_id(), young_modulus);
	return *this;
}
DeformableLineHandler& stark::models::DeformableLineHandler::set_strain_damping(const double strain_damping)
{
	this->lines->strain->set_strain_damping(this->get_id(), strain_damping);
	return *this;
}
DeformableLineHandler& stark::models::DeformableLineHandler::set_strain_limit(const double strain_limit)
{
	this->lines->strain->set_strain_limit(this->get_id(), strain_limit);
	return *this;
}
DeformableLineHandler& stark::models::DeformableLineHandler::set_strain_limit_stiffness(const double strain_limit_stiffness)
{
	this->lines->strain->set_strain_limit_stiffness(this->get_id(), strain_limit_stiffness);
	return *this;
}
DeformableLineHandler& stark::models::DeformableLineHandler::add_to_output_label(const std::string label)
{
	this->lines->output_groups.add_to_group(label, this->get_line_index());
	return *this;
}
double stark::models::DeformableLineHandler::get_linear_density() const
{
	return this->lines->inertia->get_density(this->get_id());
}
double stark::models::DeformableLineHandler::get_inertia_damping() const
{
	return this->lines->inertia->get_inertia_damping(this->get_id());
}
double stark::models::DeformableLineHandler::get_young_modulus() const
{
	return this->lines->strain->get_young_modulus(this->get_id());
}
double stark::models::DeformableLineHandler::get_strain_damping() const
{
	return this->lines->strain->get_strain_damping(this->get_id());
}
double stark::models::DeformableLineHandler::get_strain_limit() const
{
	return this->lines->strain->get_strain_limit(this->get_id());
}
double stark::models::DeformableLineHandler::get_strain_limit_stiffness() const
{
	return this->lines->strain->get_strain_limit_stiffness(this->get_id());
}
int stark::models::DeformableLineHandler::get_line_index() const
{
	return this->lines->get_index(this->get_id());
}
