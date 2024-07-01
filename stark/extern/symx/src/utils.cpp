#include "utils.h"

std::vector<symx::Scalar> symx::gather(std::vector<Vector> vectors)
{
	std::vector<symx::Scalar> variables;
	for (Vector& v : vectors) {
		for (int i = 0; i < v.size(); i++) {
			variables.push_back(v[i]);
		}
	}
	return variables;
}

void symx::reorder_in_blocks(Scalar* m, const int matrix_size, const int block_size)
{
	std::vector<Scalar> tmp;
	tmp.reserve(matrix_size * matrix_size);

	const int n_block_rows = matrix_size / block_size;
	for (int bi = 0; bi < n_block_rows; bi++) {
		for (int bj = 0; bj < n_block_rows; bj++) {

			for (int i = 0; i < block_size; i++) {
				for (int j = 0; j < block_size; j++) {
					const int row = block_size * bi + i;
					const int col = block_size * bj + j;
					const int idx = row * matrix_size + col;
					tmp.push_back(m[idx]);
				}
			}
		}
	}

	for (int i = 0; i < matrix_size * matrix_size; i++) {
		m[i] = tmp[i];
	}
}

