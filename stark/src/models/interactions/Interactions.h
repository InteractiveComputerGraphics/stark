#pragma once
#include "EnergyFrictionalContact.h"


namespace stark::models
{
	class Interactions
	{
	public:
		Interactions(core::Stark& stark, spEnergyFrictionalContact);

		// TODO: setters for friction and collision pairs

	private:
		spEnergyFrictionalContact contact = nullptr;
	};
}
