#pragma once
#include "IntervalVector.h"
#include "IntervalConnectivity.h"

#include <Eigen/Dense>

namespace stark::utils
{
	template<std::size_t N>
	class MultiMesh_
	{
	public:
		/* Fields */
		IntervalVector<Eigen::Vector3d> vertices;
		IntervalConnectivity<N> conn;

		/* Methods */
		int add(const std::vector<Eigen::Matrix<double, 3, 1>>& vertices, const std::vector<std::array<int, N>>& connectivity);
		void clear();
	};


	// DEFINITIONS ===========================================================================
	template<std::size_t N>
	inline int MultiMesh_<N>::add(const std::vector<Eigen::Matrix<double, 3, 1>>& vertices, const std::vector<std::array<int, N>>& connectivity)
	{
		const int offset = this->vertices.size();
		this->vertices.append(vertices);
		return this->conn.append(connectivity, offset);
	}
	template<std::size_t N>
	inline void MultiMesh_<N>::clear()
	{
		this->vertices.clear();
		this->conn.clear();
	}
}