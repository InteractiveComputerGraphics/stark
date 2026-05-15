#pragma once

bool project_to_PD_inplace_fixed(double* m, int n, double eps, bool mirroring);
bool project_to_PD_inplace_dynamic(double* m, int n, double eps, bool mirroring);
bool project_to_PD_inplace(double* m, int n, double eps, bool mirroring);
