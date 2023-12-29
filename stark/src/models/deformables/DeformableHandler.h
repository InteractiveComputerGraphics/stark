#pragma once

#include "Id.h"
#include "PointDynamics.h"
#include "EnergyPointInertia.h"
#include "EnergyPointPrescribedPositions.h"


namespace stark::models
{
	class DeformableHandler
	{
	private:
		Id id;
		spPointDynamics dyn = nullptr;
		spEnergyPointInertia inertia = nullptr;
		spEnergyPointPrescribedPositions prescribed_positions = nullptr;

	public:
		DeformableHandler(const Id& id, spPointDynamics dyn, spEnergyPointInertia inertia, spEnergyPointPrescribedPositions prescribed_positions);

		int get_global_idx() const;
		int get_global_vertex_idx(const int local_vertex) const;
		const std::unordered_map<std::string, int>& get_local_indices() const;

		void set_velocity(const int local_vertex, const Eigen::Vector3d& v);
		void set_position(const int local_vertex, const Eigen::Vector3d& x);
		void set_acceleration(const int local_vertex, const Eigen::Vector3d& a);
		void set_force(const int local_vertex, const Eigen::Vector3d& f);
		void clear_forces();
		void clear_accelerations();
		Eigen::Vector3d get_velocity(const int local_vertex);
		Eigen::Vector3d get_position(const int local_vertex);
		Eigen::Vector3d get_acceleration(const int local_vertex);
		Eigen::Vector3d get_force(const int local_vertex);

		std::shared_ptr<PrescribedPointGroup> create_prescribed_positions_group(const std::string label = "");
		std::shared_ptr<PrescribedPointGroupWithTransformation> create_prescribed_positions_group_with_transformation(const std::string label = "");

	protected:
		const Id& get_id() const;
	};
}
