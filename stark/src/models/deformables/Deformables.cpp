#include "Deformables.h"

using namespace stark::models;

stark::models::Deformables::Deformables(stark::core::Stark& stark, spPointDynamics dyn)
{
	auto inertia = std::make_shared<EnergyPointInertia>(stark, dyn);
	auto prescribed_positions = std::make_shared<EnergyPointPrescribedPositions>(stark, dyn);
	this->lines = std::make_shared<DeformableSolidsLines>(stark, dyn, inertia, prescribed_positions);
	this->surfaces = std::make_shared<DeformableSolidsSurfaces>(stark, dyn, inertia, prescribed_positions);
	this->volumes = std::make_shared<DeformableSolidsVolumes>(stark, dyn, inertia, prescribed_positions);
}

DeformableLineHandler stark::models::Deformables::add_line(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 2>>& segments, const MaterialLine& material)
{
	auto id = this->lines->add(vertices, segments, material);
	return DeformableLineHandler(id, this->lines);
}

DeformableSurfaceHandler stark::models::Deformables::add_surface(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const MaterialSurface& material)
{
	auto id = this->surfaces->add(vertices, triangles, material);
	return DeformableSurfaceHandler(id, this->surfaces);
}
DeformableVolumeHandler stark::models::Deformables::add_volume(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 4>>& tets, const MaterialVolume& material)
{
	auto id = this->volumes->add(vertices, tets, material);
	return DeformableVolumeHandler(id, this->volumes);
}
