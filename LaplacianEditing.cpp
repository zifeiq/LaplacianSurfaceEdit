#include "LaplacianEditing.h"
#include <Eigen/QR>

using namespace Eigen;
LaplacianEditing::LaplacianEditing(){
}


LaplacianEditing::LaplacianEditing(Mesh& m){
	int n = m.V.size();
  	spMat Adj(n,n); // adjacency matrix
  	vector<T> adj_t;
  	Eigen::MatrixXf V(m.V.size(),3);
  	Eigen::VectorXd vec_d(n);

    cout << "[Debug] Mark 1 " << endl;
  	//construct the adjacency matrix so as to construct the laplacian matrix
  	for(int i=0; i<n; i++){
    	V.row(i) << m.V[i].p[0], m.V[i].p[1], m.V[i].p[2];
    	for(int id: m.V[i].neighbors_id){
    		adj_t.push_back(T(i,id,1));
    	}
    	vec_d(i) = m.V[i].neighbors_id.size();
  	}

    cout << "[Debug] Mark 2 " << endl;

  	Adj.setFromTriplets(adj_t.begin(), adj_t.end());
  	auto mat_d = vec_d.asDiagonal();
  	auto I = Eigen::MatrixXd::Identity(n,n);
  	L = I - mat_d.inverse()*Adj;

    cout << "[Debug] Mark 3 " << endl;
  	// calculate the coefficients matrix
  	for(int i=0; i<n; i++){
  		auto v = m.V[i];
  		const int d = vec_d(i)+1;
  		vector<int> ring;
  		ring.push_back(i);
      
      Eigen::MatrixXd Cmat(3*d,7);
      Cmat.row(0) << v.p[0],0,v.p[2], -v.p[1], 1,0,0;
      Cmat.row(d) << v.p[1], -v.p[2], 0, v.p[0], 0,1,0;
      Cmat.row(2*d) << v.p[2], v.p[1], -v.p[0], 0,0,0,1;
      int j = 1;
  		for(int id: m.V[i].neighbors_id){
  			ring.push_back(id);

        Cmat.row(j) << V(id,0), 0, V(id,2), -V(id,1), 1, 0,0;
        Cmat.row(j+d) << V(id,1), -V(id,2), 0, V(id,0), 0, 1,0;
        Cmat.row(j+2*d) << V(id,2), V(id,1), -V(id,0), 0, 0, 0,1;
        j++;
  		}

      cout << "[Debug] Mark 4 " << endl;
  	
  		Eigen::CompleteOrthogonalDecomposition< Eigen::MatrixXd > cod(Cmat);
  		auto Cinv = cod.pseudoInverse();
      cout << Cmat << endl;
      cout << Cinv << endl;

  	}


}

