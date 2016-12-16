#ifndef __LAPLACIAN_SURFACE__
#define __LAPLACIAN_SURFACE__
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "Mesh.h"
using namespace std;
typedef Eigen::SparseMatrix<double> spMat;
typedef Eigen::Triplet<double> T;
class LaplacianEditing{

public:
  LaplacianEditing();
  LaplacianEditing(Mesh& m);

private:
	// The laplacian matrix
	// and A for A_prime * v_prime = [0 fixpoints]^T
  spMat L, L_prime, A, A_prime;
  // Eigen::MatrixXf V, V_prime;
};

#endif
