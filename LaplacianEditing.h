#ifndef __LAPLACIAN_SURFACE__
#define __LAPLACIAN_SURFACE__
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "Mesh.h"
using namespace std;
typedef Eigen::SparseMatrix<float, Eigen::RowMajor> spMat;
typedef Eigen::Triplet<float> T;

class LaplacianEditing{

	public:
	  LaplacianEditing();
	  void initialize(Mesh& m);
	  void translate(Vec3f v);

	private:
		// The laplacian matrix
		// and A for A_prime * v_prime = [0 fixpoints]^T  (which means rhs)
	  spMat L, L_prime, A_prime;
	  Mesh * mesh;
	  int n; // the size of the point set
	  // Eigen::MatrixXf V, V_prime;


};

#endif
