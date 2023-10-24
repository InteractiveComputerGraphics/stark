#pragma once
#include <iostream>

#include <Eigen/Dense>

namespace symx
{
	// Code from the IPC-Toolkit
	// https://github.com/ipc-sim/ipc-toolkit/blob/main/src/ipc/utils/eigen_ext.tpp
	// Matrix Projection onto Positive Semi-Definite Cone
	template <
		typename _Scalar,
		int _Rows,
		int _Cols,
		int _Options,
		int _MaxRows,
		int _MaxCols>
		Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>
		project_to_PD(
			const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& A,
			double eps = 0.0)
	{
		assert(eps >= 0);

		// https://math.stackexchange.com/q/2776803
		Eigen::SelfAdjointEigenSolver<
			Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>>
			eigensolver(A);
		if (eigensolver.info() != Eigen::Success) {
			std::cout << "SymX error: unable to project matrix onto positive definite cone" << std::endl;
			exit(-1);
		}
		// Check if all eigen values are positive.
		// The eigenvalues are sorted in increasing order.
		//if (eigensolver.eigenvalues()[0] > 0.0) {
		//	return A;
		//}
		Eigen::DiagonalMatrix<double, Eigen::Dynamic> D(eigensolver.eigenvalues());
		// Save a little time and only project the negative or zero values
		for (int i = 0; i < A.rows(); i++) {
			if (D.diagonal()[i] < eps) {
				D.diagonal()[i] = eps;
			}
			else {
				break;
			}
		}
		return eigensolver.eigenvectors() * D
			* eigensolver.eigenvectors().transpose();
	}

	inline void project_to_PD_from_pointer(double* symMtr, const int size, const bool debug_print_lowest = false)
	{
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m;
		m.resize(size, size);

		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				m(i, j) = symMtr[i*size + j];
			}
		}

		if (debug_print_lowest) {
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> eigensolver(m);
			std::vector<double> eig(eigensolver.eigenvalues().data(), eigensolver.eigenvalues().data() + size);
			if (*std::min_element(eigensolver.eigenvalues().data(), eigensolver.eigenvalues().data() + size) < 0.0) {
				std::cout << " x";
			}
			else {
				std::cout << " .";
			}
		}

		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m1 = project_to_PD(m, /*eps = */0.0);

		if (debug_print_lowest) {
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> eigensolver(m1);
			std::vector<double> eig(eigensolver.eigenvalues().data(), eigensolver.eigenvalues().data() + size);
			if (*std::min_element(eigensolver.eigenvalues().data(), eigensolver.eigenvalues().data() + size) < 0.0) {
				std::cout << "x ";
			}
			else {
				std::cout << ". ";
			}
		}

		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				symMtr[i*size + j] = m1(i, j);
			}
		}
	}
}
