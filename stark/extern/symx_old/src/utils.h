#pragma once
#include "Scalar.h"
#include "Vector.h"
#include "Matrix.h"

namespace symx
{
	void reorder_in_blocks(Scalar* m, const int matrix_size, const int block_size);
	std::vector<Scalar> gather(std::vector<Vector> vectors);
}
