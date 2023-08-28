#pragma once
#include <vector>
#include <array>

#include <Eigen/Dense>
#include <symx>
#include <TriangleMeshCollisionDetection>

#include "../utils/unordered_array_set_and_map.h"
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
		utils::unordered_array_map<int, 2, double> rb_d_mu;

		// Contacts
		tmcd::IntersectionDetection id;
		tmcd::ProximityDetection pd;
		RB_Deformable_Contacts contacts;
		RB_Deformable_Friction friction;
		
		/* Methods */
		void init(Stark& sim, Cloth* cloth, RigidBodies* rigid_bodies);
		void set_friction(const int rb_idx, const int cloth_idx, const double coulombs_mu);
		double get_friction(const int rb_idx, const int cloth_idx);
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