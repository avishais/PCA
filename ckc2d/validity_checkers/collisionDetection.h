/*
 * Checker.h
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#ifndef COLLISIONDETECTION
#define COLLISIONDETECTION

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>

#include "properties.h"

typedef std::vector<std::vector< double > > Matrix;
typedef std::vector< double > State;

using namespace std;


class collisionDetection
{
public:
	collisionDetection();
	int collision_state(State q);

	// Performance parameters
	int collisionCheck_counter;
	double collisionCheck_time;
	int get_collisionCheck_counter() {
		return collisionCheck_counter;
	}
	double get_collisionCheck_time() {
		return collisionCheck_time;
	}
	
	/** Calculate norm distance between two vectors */
	double normDistance(State, State);
	double normVector(State q);

	// Include constraints?
	const bool include_constraints = false; // Enable/Disable constraints
	bool check_angles(State, double = 1);
	bool self_collision(State, double = 1);
	bool obstacle_collision(State, double = 0.3);
	bool LinesIntersect(State A, State B, State C, State D);
	//Matrix obs = {{-4, 4, 1},{3.4, 7.5, 1}, {4.2, -2.9, 1.5}, {8, 2, 0.8}};
	Matrix obs = {{-3.2, 4, 1},{2.6, 6.8, 1}, {4.2, -2.9, 1.5}, {8, 2, 0.8}};

private:

	int n_;
	State L_;
	double qminmax_; // Joint limits


};




#endif /* COLLISIONDETECTION */
