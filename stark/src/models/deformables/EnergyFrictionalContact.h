#pragma once
#include <vector>
#include <string>
#include <memory>

#include <Eigen/Dense>
#include <symx>
#include <TriangleMeshCollisionDetection>

#include "../../solver/Energy.h"
#include "../../solver/Stark.h"
#include "../../utils/unordered_array_set_and_map.h"
#include "PointDynamics.h"
#include "../RigidBodies.h"
#include "contact_and_friction_data.h"

namespace stark::models
{
	class EnergyFrictionalContact :
		public Energy
	{
	public:
		/* Fields */
		const spPointDynamics p_dyn;
		const spRigidBodies rb;
		IntervalConnectivity<1> points;
		IntervalConnectivity<2> edges;
		IntervalConnectivity<3> triangles;
		utils::unordered_array_map<int, 2, double> pair_coulombs_mu;
		std::vector<std::string> labels;  // per group

		// Collision detection
		tmcd::IntersectionDetection id;
		tmcd::ProximityDetection pd;

		// Data structures for SymX
		Contacts_Deformables contacts_deformables;
		Contacts_RB contacts_rb;
		Contacts_RB_Deformables contacts_rb_deformables;
		Friction_Deformables friction_deformables;
		Friction_RB friction_rb;
		Friction_RB_Deformables friction_rb_deformables;


		/* Methods */
		EnergyFrictionalContact(const spPointDynamics dyn, const spRigidBodies rb);
		void declare(Stark& stark);
		int add(const int obj_idx, const std::vector<std::array<int, 3>>& triangles, const double young_modulus, const double poisson_ratio, const std::string label = "");
		void set_parameters(const int id, const double young_modulus, const double poisson_ratio);


	private:
		std::vector<symx::Vector> _get_rb_x1(const std::vector<symx::Index>& indices, const symx::Index& rb_idx, const symx::Scalar& dt, symx::Energy& energy);



	};
	using spEnergyFrictionalContact = std::shared_ptr<EnergyFrictionalContact>;
}
