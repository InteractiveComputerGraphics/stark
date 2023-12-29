#pragma once
#include "DeformableHandler.h"
#include "DeformableSolidsVolumes.h"


namespace stark::models
{
	class DeformableVolumeHandler
		: public DeformableHandler
	{
	private:
		std::shared_ptr<DeformableSolidsVolumes> volumes;

	public:
		DeformableVolumeHandler(const Id& id, std::shared_ptr<DeformableSolidsVolumes> volumes);

		// Setters
		DeformableVolumeHandler& set_density(const double density);
		DeformableVolumeHandler& set_inertia_damping(const double inertia_damping);
		DeformableVolumeHandler& set_young_modulus(const double young_modulus);
		DeformableVolumeHandler& set_poisson_ratio(const double poisson_ratio);
		DeformableVolumeHandler& set_strain_damping(const double strain_damping);
		DeformableVolumeHandler& set_strain_limit(const double strain_limit);
		DeformableVolumeHandler& set_strain_limit_stiffness(const double strain_limit_stiffness);
		DeformableVolumeHandler& add_to_output_label(const std::string label);

		// Getters
		double get_density() const;
		double get_inertia_damping() const;
		double get_young_modulus() const;
		double get_poisson_ratio() const;
		double get_strain_damping() const;
		double get_strain_limit() const;
		double get_strain_limit_stiffness() const;
		int get_volume_index() const;
	};
}
