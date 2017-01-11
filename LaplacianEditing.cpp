#include "LaplacianEditing.h"
#include <Eigen/QR>
#include <fstream>
using namespace Eigen;
LaplacianEditing::LaplacianEditing(){
}


void LaplacianEditing::initialize(Mesh& m){
    mesh = &m;
    // n = m.V.size();
    n = m.interests.size();
  	spMat Adj(n,n); // adjacency matrix
  	vector<T> adj_t;
  	// Eigen::MatrixXf V(m.V.size(),3);
    Eigen::MatrixXf V(m.interests.size(),3);
  	Eigen::VectorXf vec_d(n);

    // update the vertex id according to their position in the ROI
    for(int i=0; i<m.interests.size(); i++){
      m.interests[i]->id = i;
    }

  	//construct the adjacency matrix so as to construct the laplacian matrix
  	for(int i=0; i<n; i++){
      vec_d(i) = 0;
    	V.row(i) << m.interests[i]->p[0], m.interests[i]->p[1], m.interests[i]->p[2];
    	for(auto vv: m.interests[i]->neighbors){
        if(vv->isSelected){
          adj_t.push_back(T(i,vv->id,1));  
          vec_d(i)++;
        }
    	}
    	// vec_d(i) = m.interests[i]->neighbors.size();
  	}

  	Adj.setFromTriplets(adj_t.begin(), adj_t.end());

    // MatrixXf d_Adj = MatrixXf(Adj);
    // cout << d_Adj << endl;
  	MatrixXf mat_d = vec_d.asDiagonal();
  	// auto I = Eigen::MatrixXf::Identity(n,n);
    cout << "[Debug] Constructing sparse identity matrix ..." << flush;
    spMat I(n,n);
    I.setIdentity();
    cout << "Finished " << endl;
  	L = I - mat_d.inverse()*Adj;

    MatrixXf dV = L*V;
    cout << "[Debug] Finish the intial construction of L" << endl;

    L_prime.resize(3*L.rows(), 3*L.cols());
    int r = L.rows();
    int c = L.cols();

    vector<T> lprime_t;
    for(int k=0; k<L.outerSize(); k++)
      for(spMat::InnerIterator it(L,k); it; ++it){
        // cout << it.row() << "," << it.col() << endl;
        L_prime.insert(it.row(),it.col()) = it.value();
        L_prime.insert(it.row()+r,it.col()+c) = it.value();
        L_prime.insert(it.row()+2*r,it.col()+2*c) = it.value();
      }

    cout << "[Debug] Finish the intial construction of L_prime" << endl;

  	// calculate the coefficients matrix
  	for(int i=0; i<n; i++){
      // cout << "[Debug] Calculating the coefficients matrix ..." << i << endl;
  		const int d = vec_d(i)+1;
  		vector<int> ring;
  		ring.push_back(i);

      // cout << "[Debug] debug mark 1" << endl;
  		for(auto vv: m.interests[i]->neighbors){
        if(vv->isSelected)
  			 ring.push_back(vv->id);
  		}

  	  int j =0;
      Eigen::MatrixXf Cmat(3*d,7);
      for(int id: ring){
        Cmat.row(j) << V(id,0), 0, V(id,2), -V(id,1), 1, 0,0;
        Cmat.row(j+d) << V(id,1), -V(id,2), 0, V(id,0), 0, 1,0;
        Cmat.row(j+2*d) << V(id,2), V(id,1), -V(id,0), 0, 0, 0,1;
        j++;
      }
      // cout << "[Debug] debug mark 2" << endl;
  		Eigen::CompleteOrthogonalDecomposition< Eigen::MatrixXf > cod(Cmat);
  		// auto Cinv = cod.pseudoInverse();
      MatrixXf Cinv(7,3*d);
      JacobiSVD<MatrixXf> svd(Cmat, ComputeThinV | ComputeThinU);
      svd.pinv(Cinv);
      // cout << "Cmat " << Cmat.rows() << "," << Cmat.cols() << endl;
      // cout << "Cinv " << Cinv.rows() << "," << Cmat.cols() << endl; 
      // cout << "[Debug] debug mark 3" << endl;
      ofstream fcinv;
      fcinv.open("cinv.txt");
      fcinv << Cinv;
      fcinv.close();

      VectorXf s = Cinv.row(0);
      VectorXf h1 = Cinv.row(1);
      VectorXf h2 = Cinv.row(2);
      VectorXf h3 = Cinv.row(3);

      double dx = dV(i,0);
      double dy = dV(i,1);
      double dz = dV(i,2);

      MatrixXf Tdelta(3,3*d);

      // cout << "[Debug] debug mark 3.5" << endl;
      // cout << s.size() << endl;
      // cout << h1.size() << endl;
      // cout << h2.size() << endl;
      Tdelta.row(0) = dx*s - dy*h3 + dz*h2;
      // cout << "[Debug] debug mark 3.6" << endl;
      Tdelta.row(1) = dx*h3 + dy*s - dz*h1;
      Tdelta.row(2) = -dx*h2 + dy*h1 +dz*s;

      // cout << "[Debug] debug mark 4" << endl;

      for(int j=0; j<ring.size(); j++){
        int r = ring[j];
        L_prime.coeffRef(i,r) -= Tdelta(0,j);
        L_prime.coeffRef(i,r+n) -= Tdelta(0,j+d);
        L_prime.coeffRef(i,r+2*n) -= Tdelta(0,j+2*d);

        L_prime.coeffRef(i+n,r) -= Tdelta(1,j);
        L_prime.coeffRef(i+n,r+n) -= Tdelta(1,j+d);
        L_prime.coeffRef(i+n,r+2*n) -= Tdelta(1,j+2*d);

        L_prime.coeffRef(i+2*n,r) -= Tdelta(2,j);
        L_prime.coeffRef(i+2*n,r+n) -= Tdelta(2,j+d);
        L_prime.coeffRef(i+2*n,r+2*n) -= Tdelta(2,j+2*d);
      } 
      // cout << "[Debug] debug mark 5" << endl;

  	}
    cout << "[Debug] Laplacian editing initialization finished" << endl;
}


void LaplacianEditing::translate(Vec3f v){

  // calculate the center of the handle
  cout << "[Debug]Calculating the center of the handle ..." << flush;
  Vec3f centerH(0.0,0.0,0.0);
  for(int i=0; i<mesh->handle.size(); i++){
    Vec3f tv = mesh->handle[i]->p;
    centerH+=tv;
  }
  centerH/=mesh->handle.size();

  cout << "Finished" << endl;

  // the translation
  Vec3f trans = v - centerH;
  cout << "The translation " << endl;
  cout << trans << endl;

  trans[2] = 0.1;

  cout << "[Debug] Apply the translation to the handle ... " << flush;
  // add the translation to all the points of handle
  for(int i=0; i<mesh->handle.size(); i++){
    mesh->handle[i]->p+=trans;
  }

  cout << "Finished" << endl;

  cout << "[Debug] Constructing A_prime ... " << flush;
  A_prime.resize(L_prime.rows()+3*mesh->handle.size()+3*mesh->anchor.size(), L_prime.cols());
  // A_prime.topRows(L_prime.rows()) = L_prime;
  for(int k=0; k<L_prime.outerSize(); k++)
    for(spMat::InnerIterator it(L_prime,k); it; ++it){
      A_prime.insert(it.row(),it.col()) = it.value();
    }
  int lr = L_prime.rows();
  // VectorXf rhs(3*n+3*(mesh->anchor.size()+mesh->handle.size()));
  SparseVector<float> rhs(3*n+3*(mesh->anchor.size()+mesh->handle.size()));
  for(int i=0; i<mesh->anchor.size(); i++){
    A_prime.insert(3*i+lr, mesh->anchor[i]->id) = 1;
    A_prime.insert(3*i+lr+1, mesh->anchor[i]->id+n) = 1;
    A_prime.insert(3*i+lr+2, mesh->anchor[i]->id+2*n) = 1;

    rhs.insert(3*n+3*i) = mesh->anchor[i]->p[0];
    rhs.insert(3*n+3*i+1) = mesh->anchor[i]->p[1];
    rhs.insert(3*n+3*i+2) = mesh->anchor[i]->p[2];
  }

  cout << "Anchor finished ..." << flush;
  lr += 3*mesh->anchor.size();
  for(int i=0; i<mesh->handle.size(); i++){
    A_prime.insert(3*i+lr, mesh->handle[i]->id) = 1;
    A_prime.insert(3*i+lr+1, mesh->handle[i]->id+n) = 1;
    A_prime.insert(3*i+lr+2, mesh->handle[i]->id+2*n) = 1;

    rhs.insert(3*(n+mesh->anchor.size())+3*i) = mesh->handle[i]->p[0];
    rhs.insert(3*(n+mesh->anchor.size())+3*i+1) = mesh->handle[i]->p[1];
    rhs.insert(3*(n+mesh->anchor.size())+3*i+2) = mesh->handle[i]->p[2];
  }

  cout << "Handle finished " << endl;
  A_prime.makeCompressed();

  ofstream faprime;
  faprime.open("A_prime.txt");
  for(int k=0; k<A_prime.outerSize(); k++)
    for(spMat::InnerIterator it(A_prime,k); it; ++it){
      // cout << it.row() << "," << it.col() << endl;
      faprime << it.row() << " " << it.col() << " " << it.value() << endl;
    }
  faprime.close();

  cout << "[Debug] Solving the linear system ..." << endl;
  SparseQR<spMat,COLAMDOrdering<int>  > solver;
  // solver.setPivotThreshold(0.0f);
  // SimplicialLDLT<spMat> solver;
  // SparseLU<spMat> solver;
  solver.compute(A_prime);
  cout << "Compute decomposition finished ... " << flush;
  

  MatrixXf Q = solver.matrixQ();
  spMat R = solver.matrixR();
  cout << "Saving Q " << Q.size() << "... " << flush;
  ofstream fq;
  fq.open("q.txt");
  // for(int k=0; k<Q.outerSize(); k++)
  //   for(spMat::InnerIterator it(Q,k); it; ++it){
  //     // cout << it.row() << "," << it.col() << endl;
  //     fq << it.row() << " " << it.col() << " " << it.value() << endl;
  //   }
  fq << Q;
  fq.close();
  cout << "Finished" << endl;
  cout << "Saving R " << R.size() << "... "<<flush;
  ofstream fr;
  fr.open("r.txt");
  for(int k=0; k<R.outerSize(); k++)
    for(spMat::InnerIterator it(R,k); it; ++it){
      // cout << it.row() << "," << it.col() << endl;
      fr << it.row() << " " << it.col() << " " << it.value() << endl;
    }
  fr.close();
  cout << "Finished" << endl;
  VectorXf v_new = solver.solve(rhs);
  // v_new.normalize();
  // MatrixXf A_prime_d(A_prime);
  // auto v_new = (A_prime_d.transpose()*A_prime_d).ldlt().solve(A_prime_d.transpose()*rhs);
  cout << "Linear system solving finished" << endl;
  // auto v_new = A_prime.jacobiSvd(ComputeThinU|ComputeThinV).solve(rhs);
  cout << "[Debug] Apply the result ..." << flush;
  for(int i=0; i<mesh->interests.size(); i++){
    if(i == mesh->V.size()/10) cout << "10%% " << flush;
    if(i == mesh->V.size()/5) cout << "20%% " << flush;
    if(i == mesh->V.size()*3/10) cout << "30%% " << flush;
    if(i == mesh->V.size()*2/5) cout << "40%% " << flush;
    if(i == mesh->V.size()/2) cout << "50%% " << flush;
    if(i == mesh->V.size()*3/5) cout << "60%% " << flush;
    if(i == mesh->V.size()*7/10) cout << "70%% " << flush;
    if(i == mesh->V.size()*4/5) cout << "80%% " << flush;
    if(i == mesh->V.size()*9/10) cout << "90%% " << flush;

    // mesh->V[i].p = Vec3f(v_new(3*i), v_new(3*i+1), v_new(3*i+2));
    mesh->interests[i]->p = Vec3f(v_new(i), v_new(i+n), v_new(i+2*n));
  }
  // mesh->recomputeNormals();
  // mesh->centerAndScaleToUnit();
  cout << "Finished" << endl;
  cout << "Saving the result into file ..." << flush;
  ofstream fresult;
  fresult.open("result.txt");
  for(auto p: mesh->V){
    fresult << p.p[0] << "," << p.p[1] << "," << p.p[2] << endl;
  }
  fresult.close();
  cout << "Finished" << endl;
}
