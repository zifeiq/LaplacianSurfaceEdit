// --------------------------------------------------------------------------
// Copyright(C) 2009-2015
// Tamy Boubekeur
//                                                                            
// All rights reserved.                                                       
//                                                                            
// This program is free software; you can redistribute it and/or modify       
// it under the terms of the GNU General Public License as published by       
// the Free Software Foundation; either version 2 of the License, or          
// (at your option) any later version.                                        
//                                                                            
// This program is distributed in the hope that it will be useful,            
// but WITHOUT ANY WARRANTY; without even the implied warranty of             
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              
// GNU General Public License (http://www.gnu.org/licenses/gpl.txt)           
// for more details.                                                          
// --------------------------------------------------------------------------

#include "Mesh.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <queue>

using namespace std;

void Mesh::loadOFF (const std::string & filename) {
	ifstream in (filename.c_str ());
    if (!in) 
        exit (1);
	string offString;
    unsigned int sizeV, sizeT, tmp;
    in >> offString >> sizeV >> sizeT >> tmp;
    V.resize (sizeV);
    T.resize (sizeT);
    for (unsigned int i = 0; i < sizeV; i++)
        in >> V[i].p;
    int s;
    for (unsigned int i = 0; i < sizeT; i++) {
        in >> s;
        for (unsigned int j = 0; j < 3; j++)
            in >> T[i].v[j];
    }
    in.close ();
    centerAndScaleToUnit ();
    recomputeNormals ();
}

void Mesh::recomputeNormals () {
    for (unsigned int i = 0; i < V.size (); i++)
        V[i].n = Vec3f (0.0, 0.0, 0.0);
    for (unsigned int i = 0; i < T.size (); i++) {
        Vec3f e01 = V[T[i].v[1]].p -  V[T[i].v[0]].p;
        Vec3f e02 = V[T[i].v[2]].p -  V[T[i].v[0]].p;
        Vec3f n = cross (e01, e02);
        n.normalize ();
        for (unsigned int j = 0; j < 3; j++)
            V[T[i].v[j]].n += n;
    }
    for (unsigned int i = 0; i < V.size (); i++)
        V[i].n.normalize ();
}

void Mesh::centerAndScaleToUnit () {
    Vec3f c;
    for  (unsigned int i = 0; i < V.size (); i++)
        c += V[i].p;
    c /= V.size ();
    float maxD = dist (V[0].p, c);
    for (unsigned int i = 0; i < V.size (); i++){
        float m = dist (V[i].p, c);
        if (m > maxD)
            maxD = m;
    }
    for  (unsigned int i = 0; i < V.size (); i++)
        V[i].p = (V[i].p - c) / maxD;
}

// written by Zifei QIAN
void Mesh::loadOBJ(const std::string & filename){
    cout << "[Debug] Loading OBJ File " << filename << endl;
    ifstream in (filename.c_str ());
    if (!in) 
        exit (1);
    string line;
    Vec3f vp(0,0,0);
    while(getline(in,line)){
        istringstream iss(line.substr(1));
        switch(line[0]){
            case '#':
            break;
            case 'v':
            V.push_back(Vertex());
            iss >> V.back().p[0] >> V.back().p[1] >> V.back().p[2] ;
            if(V.back().p[1]>vp[1]) vp = V.back().p;
            break;
            case 'f':
            T.push_back(Triangle());
            iss >> T.back().v[0] >> T.back().v[1] >> T.back().v[2];
            T.back().v[0]-=1;
            T.back().v[1]-=1;
            T.back().v[2]-=1;
            addNeighbor(T.back());
            break;
            default:
            break;
        }

    }
    cout << "[Debug] The point with biggest y is : (" << vp[0] << "," << vp[1] << "," << vp[2] << endl;
    in.close ();
    centerAndScaleToUnit ();
    recomputeNormals ();
}

void Mesh::addNeighbor(Triangle& t){
    V[t.v[0]].addNeighbor(&V[t.v[1]]);
    V[t.v[0]].addNeighbor(&V[t.v[2]]);
    V[t.v[1]].addNeighbor(&V[t.v[0]]);
    V[t.v[1]].addNeighbor(&V[t.v[2]]);
    V[t.v[2]].addNeighbor(&V[t.v[0]]);
    V[t.v[2]].addNeighbor(&V[t.v[1]]);
}

void Mesh::selectPart(Vec3f p, float range, bool selectMode){
    float minD = 100;
    int selectedV;
    for(int i=0; i<V.size(); i++){
        float d = dist(V[i].p,p);
        if(d<minD){
            minD = d;
            selectedV = i; 
        }
    }
    cout << "Min distance: " << minD << endl;
    if(minD > 5) return;
    queue<Vertex*> q;
    q.push(&V[selectedV]);
    while(!q.empty()){
        Vertex* v = q.front();
        q.pop();
        v->isSelected = true;
        if(selectMode){
            interests.push_back(v);    
        }
        else{
            v->isHandle = true;
            handle.push_back(v);
        }
        
        for(int i=0; i<v->neighbors.size(); i++){
            if(dist(v->neighbors[i]->p, V[selectedV].p)<range){
                if(!v->neighbors[i]->isSelected ){
                    q.push(v->neighbors[i]);
                }
            }
            else{
                anchor.push_back(v);
            }
        }
    }


}

void Mesh::laplacianTransform(Vec3f v_prime){
}