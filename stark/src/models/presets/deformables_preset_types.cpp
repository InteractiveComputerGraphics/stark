#include "deformables_preset_types.h"

#include "../../utils/include.h"

/*
*	Note on damping:
*		All models have a bit of damping by default to avoid purely elastic oscillations.
*		The default is 10% of the effective stiffness.
*		However, keep in mind that some objects are more susceptible to air resistance than others (e.g. cloth).
*		The idea of a default value is to be non-intrusive. Keep an eye on it and adjust it to your needs.
* 
*	Note on contact:
*		There is no good default for contact thickness, it must be manually set always.
*		Contact thickness is set to -1.0 by default, which will throw an error.
*/

stark::Line::Params::Params(const Preset& preset)
{
	switch (preset)
	{
	case Preset::Elastic_Rubberband:
		this->inertia.density = 0.05;  // [kg/m^2]
		this->inertia.damping = 0.1;
		this->strain.elasticity_only = false;
		this->strain.section_radius = 0.002;  // [m]
		this->strain.youngs_modulus = 1e4;
		this->strain.strain_limit = std::numeric_limits<double>::max();
		this->strain.strain_limit_stiffness = 1e5;
		this->strain.damping = 1e-4;
		// Contact is left as default
		break;
	}
}
stark::Line::Params stark::Line::Params::Elastic_Rubberband()
{
	return Params(Preset::Elastic_Rubberband);
}


stark::Surface::Params::Params(const Preset& preset)
{
	switch (preset)
	{
	case Preset::Cotton_Fabric:
		this->inertia.density = 0.2;  // [kg/m^2]
		this->inertia.damping = 0.1;
		this->strain.elasticity_only = false;
		this->strain.thickness = 0.001;  // [m]
		this->strain.youngs_modulus = 1e3;
		this->strain.poissons_ratio = 0.3;
		this->strain.strain_limit = 0.1;
		this->strain.strain_limit_stiffness = 1e6;
		this->strain.damping = 0.1*this->strain.thickness*this->strain.youngs_modulus;
		this->bending.flat_rest_angle = true;  // Cloth is usually manufactured from flat pieces stitched together. A Tshirt does not tend to keep its Tshirt shape.
		this->bending.stiffness = 1e-6;
		this->bending.damping = 0.1*this->bending.stiffness;
		// Contact is left as default
		break;
	}
}
stark::Surface::Params stark::Surface::Params::Cotton_Fabric()
{
	return Params(Preset::Cotton_Fabric);
}

stark::Volume::Params::Params(const Preset& preset)
{
	switch (preset)
	{
	case Preset::Soft_Rubber:
		this->inertia.density = 1000.0;  // [kg/m^3]
		this->inertia.damping = 0.1;
		this->strain.elasticity_only = false;
		this->strain.youngs_modulus = 1e4;
		this->strain.poissons_ratio = 0.3;  // Almost incompressible
		this->strain.strain_limit = 1.0;  // Can stretch 100% of its original size before extra stiffening
		this->strain.strain_limit_stiffness = 1e2;  // Relatively soft Extra stiffening
		this->strain.damping = 0.0;
		// Contact is left as default
		break;
	}
}
stark::Volume::Params stark::Volume::Params::Soft_Rubber()
{
	return Params(Preset::Soft_Rubber);
}