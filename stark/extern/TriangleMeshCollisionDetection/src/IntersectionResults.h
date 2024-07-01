#pragma once
#include <vector>

#include "types.h"


namespace tmcd
{
	struct IntersectionResults
	{
		std::vector<std::pair<Edge, Triangle>> edge_triangle;
		void clear()
		{
			this->edge_triangle.clear();
		};
	};
}