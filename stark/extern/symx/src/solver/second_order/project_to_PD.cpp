#include "project_to_PD.h"

#include <iostream>
#include <Eigen/Dense>


// ---------------------------------------------------------------------------
// internal template for compile‑time N
// ---------------------------------------------------------------------------
namespace detail {

    template <typename Derived>
    bool project(Eigen::MatrixBase<Derived>& A, double eps, bool mirroring)
    {
        using PlainMatrixType = typename Derived::PlainObject;
        Eigen::SelfAdjointEigenSolver<PlainMatrixType> eig(A.derived());
        if (eig.info() != Eigen::Success) return false;  // ill‑conditioned – give up

        auto values = eig.eigenvalues();
		const int size = (int)values.size();
        bool changed = false;
        for (int i = 0; i < size; ++i) {
            if (values[i] < eps) {
                changed = true;
                values[i] = mirroring ? -values[i] : eps;
            }
        }
        if (changed) {
            A = eig.eigenvectors() * values.asDiagonal() * eig.eigenvectors().transpose();
        }
        return changed;
    }

    template <int N>
    bool project_fixed(double* ptr, double eps, bool mirroring)
    {
        using Mat = Eigen::Matrix<double, N, N, Eigen::RowMajor>;
        Eigen::Map<Mat> A(ptr);
		return project(A, eps, mirroring);
    }

    bool project_dynamic(double* ptr, int size, double eps, bool mirroring)
    {
        using Mat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        Eigen::Map<Mat> A(ptr, size, size);
		return project(A, eps, mirroring);
    }

} // namespace detail

// ---------------------------------------------------------------------------
// public dispatch
// ---------------------------------------------------------------------------
bool project_to_PD_inplace_fixed(double* m, int n, double eps, bool mirroring)
{
    switch (n) {
    case 3:  return detail::project_fixed<3>(m, eps, mirroring);
    case 6:  return detail::project_fixed<6>(m, eps, mirroring);
    case 9:  return detail::project_fixed<9>(m, eps, mirroring);
    case 12: return detail::project_fixed<12>(m, eps, mirroring);
    case 15: return detail::project_fixed<15>(m, eps, mirroring);
    //case 18: return detail::project_fixed<18>(m, eps, mirroring);
    default: 
		std::cout << "symx error: project_to_PD_inplace_fixed() unsupported size " << n << std::endl;
		exit(-1);
    }
}

bool project_to_PD_inplace_dynamic(double* m, int size, double eps, bool mirroring)
{
    return detail::project_dynamic(m, size, eps, mirroring);
}

bool project_to_PD_inplace(double* m, int n, double eps, bool mirroring)
{
    if (n <= 15) {
		return project_to_PD_inplace_fixed(m, n, eps, mirroring);
	}
	else {
		return project_to_PD_inplace_dynamic(m, n, eps, mirroring);
	}
}
