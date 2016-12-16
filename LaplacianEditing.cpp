#include "LaplacianEditing.h"
#include <Eigen/QR>
LaplacianEditing::LaplacianEditing(){
}


LaplacianEditing::LaplacianEditing(Mesh& m){
	int n = m.V.size();
  	spMat Adj(n,n); // adjacency matrix
  	vector<T> adj_t;
  	Eigen::MatrixXf V(m.V.size(),3);
  	Eigen::VectorXd vec_d(n);

  	//construct the adjacency matrix so as to construct the laplacian matrix
  	for(int i=0; i<n; i++){
    	V << m.V[i].p[0], m.V[i].p[1], m.V[i].p[2];
    	for(int id: m.V[i].neighbors_id){
    		adj_t.push_back(T(i,id,1));
    	}
    	vec_d(i) = m.V[i].neighbors_id.size();
  	}

  	Adj.setFromTriplets(adj_t.begin(), adj_t.end());
  	auto mat_d = vec_d.asDiagonal();
  	auto I = Eigen::MatrixXd::Identity(n,n);
  	L = I - mat_d.inverse()*Adj;


  	// calculate the coefficients matrix
  	for(int i=0; i<n; i++){
  		auto v = m.V[i];
  		const int d = vec_d(i)+1;
  		Eigen::MatrixXd Cmatx(d,7);
  		Eigen::MatrixXd Cmaty(d,7);
  		Eigen::MatrixXd Cmatz(d,7);
  		vector<int> ring;
  		ring.push_back(i);
  		for(int id: m.V[i].neighbors_id){
  			ring.push_back(id);

  			Cmatx << V(i,0), 0, V(i,2), -V(i,1), 1, 0,0;
  			Cmaty << V(i,1), -V(i,2), 0, V(i,0), 0, 1,0;
  			Cmatz << V(i,2), V(i,1), -V(i,0), 0, 0, 0,1;
  		}

  		Eigen::MatrixXd Cmat(3*d,7);
  		Cmat << Cmatx, Cmaty, Cmatz;
  		Eigen::CompleteOrthogonalDecomposition< Eigen::MatrixXd > cod(Cmat);
  		auto Cinv = cod.pseudoInverse();

  	}

}

