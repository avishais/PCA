#ifndef COLLISIONDETECTION
#define COLLISIONDETECTION 

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "PQP.h"
#include "model.h"
#include "MatVec.h"
#include <iostream>
#include <string>
#include <vector>

#include "../properties.h"

typedef std::vector<std::vector< double > > Matrix;
typedef std::vector< double > State;

class collisionDetection
{
public:

	double offsetX, offsetY, offsetZ, offsetRot;
	collisionDetection();
	void load_models();
	int collision_state(State q);

	PQP_Model base, link1, link2, link3, link4, link5, link6, EE, table, obs1, obs2, obs3;
	PQP_Model base2, link12, link22, link32, link42, link52, link62, EE2, rod;

	// Performance parameters
	int collisionCheck_counter;
	double collisionCheck_time;
	int get_collisionCheck_counter() {
		return collisionCheck_counter;
	}
	double get_collisionCheck_time() {
		return collisionCheck_time;
	}

	int env;

	double L; // Rod length
	
	Matrix Q;
	Matrix P;

	/** Return transformation matrix of rod end-tip in rod coordinate frame (at the other end-point) */
	Matrix getQ() {
		return Q;
	}

	/** Set transformation matrix of rod end-tip in rod coordinate frame (at the other end-point) */
	void setQ() {
		State v(4);

		v = {1,0,0,L};
		Q.push_back(v);
		v = {0,1,0,0};
		Q.push_back(v);
		v = {0,0,1,0};
		Q.push_back(v);
		v = {0,0,0,1};
		Q.push_back(v);
	}

	/** Set matrix of coordinated along the rod (in rod coordinate frame) */
	void setP() {
		State v(3);
		int dl = 20;
		int n = L / dl;
		for (int i = 0; i <= n; i++) {
			v = {(double)i*dl,0,0};
			P.push_back(v);
		}
	}

	/** Return matrix of coordinated along the rod (in rod coordinate frame) */
	Matrix getPMatrix() {
		return P;
	}
};

#endif
