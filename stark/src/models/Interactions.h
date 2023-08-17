#pragma once
#include <vector>
#include <array>

#include <Eigen/Dense>
#include <symx>
#include <TriangleMeshCollisionDetection>

#include "../solver/Stark.h"
#include "Cloth.h"
#include "RigidBodies.h"
#include "RB_Deformable_Contacts.h"
#include "RB_Deformable_Friction.h"


namespace stark::models
{
	class Interactions
	{
	public:
		/* Fields */
		Cloth* cloth = nullptr;
		RigidBodies* rigid_bodies = nullptr;
		int cloth_id = -1;
		int rigid_bodies_id = -1;

		// Contacts
		tmcd::IntersectionDetection id;
		tmcd::ProximityDetection pd;
		RB_Deformable_Contacts contacts;
		RB_Deformable_Friction friction;
		
		/* Methods */
		void init(Stark& sim, Cloth* cloth, RigidBodies* rigid_bodies);
		bool is_empty() const;

	private:
		// Helpers
		const tmcd::ProximityResults& _run_proximity_detection(const std::vector<Eigen::Vector3d>& x_cloth, const std::vector<Eigen::Vector3d>& x_rb, Stark& sim);
		void _update_contacts(Stark& sim);

		// Stark callbacks
		void _update_friction_contacts(Stark& sim);
		bool _is_valid_configuration(Stark& sim);

		// Energy groups
		void _energies_contact(Stark& sim);
		void _energies_friction(Stark& sim);
	};
}