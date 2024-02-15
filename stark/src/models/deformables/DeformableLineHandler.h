#pragma once
#include "DeformableHandler.h"
#include "DeformableSolidsLines.h"


namespace stark::models
{
	class DeformableLineHandler
		: public DeformableHandler
	{
	private:
		std::shared_ptr<DeformableSolidsLines> lines;

	public:
		DeformableLineHandler(const Id& id, std::shared_ptr<DeformableSolidsLines> lines);

		// Setters
		DeformableLineHandler& set_linear_density(const double density);
		DeformableLineHandler& set_inertia_damping(const double inertia_damping);
		DeformableLineHandler& set_young_modulus(const double young_modulus);
		DeformableLineHandler& set_strain_damping(const double strain_damping);
		DeformableLineHandler& set_strain_limit(const double strain_limit);
		DeformableLineHandler& set_strain_limit_stiffness(const double strain_limit_stiffness);
		DeformableLineHandler& add_to_output_label(const std::string label);

		// Getters
		double get_linear_density() const;
		double get_inertia_damping() const;
		double get_young_modulus() const;
		double get_strain_damping() const;
		double get_strain_limit() const;
		double get_strain_limit_stiffness() const;
		int get_line_index() const;
	};
}
