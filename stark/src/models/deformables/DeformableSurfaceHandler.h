#pragma once

#include "DeformableHandler.h"
#include "DeformableSolidsSurfaces.h"


namespace stark::models
{
	class DeformableSurfaceHandler
		: public DeformableHandler
	{
	private:
		std::shared_ptr<DeformableSolidsSurfaces> surfaces;

	public:
		DeformableSurfaceHandler(const Id& id, std::shared_ptr<DeformableSolidsSurfaces> surfaces);

		// Setters
		DeformableSurfaceHandler& set_area_density(const double density);
		DeformableSurfaceHandler& set_thickness(const double thickness);
		DeformableSurfaceHandler& set_inertia_damping(const double inertia_damping);
		DeformableSurfaceHandler& set_young_modulus(const double young_modulus);
		DeformableSurfaceHandler& set_poisson_ratio(const double poisson_ratio);
		DeformableSurfaceHandler& set_strain_damping(const double strain_damping);
		DeformableSurfaceHandler& set_strain_limit(const double strain_limit);
		DeformableSurfaceHandler& set_strain_limit_stiffness(const double strain_limit_stiffness);
		DeformableSurfaceHandler& set_bending_stiffness(const double bending_stiffness);
		DeformableSurfaceHandler& set_bending_damping(const double bending_damping);
		DeformableSurfaceHandler& add_to_output_label(const std::string label);

		// Getters
		double get_area_density() const;
		double get_thickness() const;
		double get_inertia_damping() const;
		double get_young_modulus() const;
		double get_poisson_ratio() const;
		double get_strain_damping() const;
		double get_strain_limit() const;
		double get_strain_limit_stiffness() const;
		double get_bending_stiffness() const;
		double get_bending_damping() const;
		int get_surface_index() const;
	};
}
