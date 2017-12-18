/*
 * Checker.h
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#ifndef CHECKER_H_
#define CHECKER_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/State.h"
#include <ompl/config.h>

#include "projectC.h"
#include "collisionDetection.h"
//#include "../../utils/pca.h"

#include <iostream>

#define PCA_KNN 20

namespace ob = ompl::base;
using namespace std;

class StateValidityChecker : public collisionDetection, public projectC//, public stats::pca
{
public:
	/** Constructors */
	StateValidityChecker(const ob::SpaceInformationPtr &si) :
		n(7),
		mysi_(si.get())
		//stats::pca(n)
			{q_prev.resize(n);
				set_n(n);
			//set_do_bootstrap(false, 100);
			}; //Constructor
	StateValidityChecker() :
		n(7)
		//stats::pca(n)
			{q_prev.resize(n);
				set_n(n);
			//set_do_bootstrap(false, 100);
			}; //Constructor

	/** Validity check using standard OMPL */
	bool isValid(const ob::State *);

	/** Validity check for a vector<double> type  */
	bool isValidRBS(State&);

	/** Serial local connection check  */
	bool checkMotion(const ob::State *, const ob::State *);

	/** Recursive Bi-Section local connection check  */
	bool checkMotionRBS(const ob::State *, const ob::State *);
	bool checkMotionRBS(State, State, int, int);

	/** Reconstruct a local connection using RBS for post-processing  */
	bool reconstructRBS(const ob::State *, const ob::State *, Matrix &);
	bool reconstructRBS(State, State, Matrix &, int, int, int);

	/** Sewing local connection check */
	bool isValidSew(State&);
	bool checkMotionSew(const ob::State *, const ob::State *);

	/** Reconstruct a local connection using Sewing strategy for post-processing  */
	bool reconstructSew(const ob::State *, const ob::State *, Matrix &);
	bool reconstructSew(State, State, Matrix &);

	/** Retrieve state from ob::State to vector<double> */
	void retrieveStateVector(const ob::State *, State &);

	/** Update state to ob::State from vector<double> */
	void updateStateVector(const ob::State *, State);

	/** Print ob::State ro console */
	void printStateVector(const ob::State *state);

	/** Set default OMPL setting */
	void defaultSettings();

	/** Calculate norm distance between two vectors */
	double normDistance(State, State);
	double stateDistance(const ob::State*, const ob::State*);

	/** Max distance between two vectors */
	double maxDistance(State, State);
	double MaxAngleDistance(State, State);

	/** Project a configuration in the ambient space to the constraint surface (and check collisions and joint limits) */
	bool IKproject(const ob::State *, bool = true);
	bool IKproject(State &, bool = true);

	/** Sample a random configuration */
	State sample_q();

	/** Join the two robots joint vectors */
	void Join_States(State&, State, State);

	/** Decouple the two robots joint vectors */
	void seperate_Vector(State, State &, State &);

	int get_valid_solution_index() {
		return valid_solution_index;
	}

	// /** Sample based on pca of nearest neighbors */
	// State sample_pca(Matrix, int = 6);

	// /** Reconstruct a vector in the pca subspace to the C-space */
	// State reconstruct_pca(State);

	// /** Find the number of dimensions that preseve a certain percent of the information */
	// int get_dim_pca();		

	/** Performance parameters measured during the planning */
	int isValid_counter;
	int get_isValid_counter() {
		return isValid_counter;
	}

	int get_n() {
		return n;
	}
	double get_RBS_tol() {
		return RBS_tol;
	}

	void set_q_prev(State q) {
		for (int i = 0; i < q.size(); i++)
			q_prev[i] = q[i];
	}

	void log_q(State q, bool New = true);

	// Performance parameters and handle
	double total_runtime; // Total planning time
	clock_t startTime; // Start clock
	clock_t endTime; // End clock
	int nodes_in_path; // Log nodes in path
	int nodes_in_trees; // Log nodes in both trees
	double PlanDistance; // Norm distance from start to goal configurations
	bool final_solved; // Planning query solved?
	double local_connection_time; // Log LC total time
	int local_connection_count; // Log number of LC attempts
    int local_connection_success_count; // Log number of LC success
    double sampling_time;
    State sampling_counter;

	/** Reset log parameters */
	void initiate_log_parameters() {
		IK_counter = 0;
		IK_time = 0;
		collisionCheck_counter = 0;
		collisionCheck_time = 0;
		isValid_counter = 0;
		nodes_in_path = 0;
		nodes_in_trees = 0;
		local_connection_time = 0;
		local_connection_count = 0;
    	local_connection_success_count = 0;
    	sampling_time = 0;
    	sampling_counter.resize(2);
    	sampling_counter[0] = sampling_counter[1] = 0; // [0/1] - successful/failed sampling
	}

	void LogPerf2file();

private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;
	int valid_solution_index;
	State q_prev;

	double dq = 0.05; // Serial local connection resolution
	bool withObs = false; // Include obstacles?
	double RBS_tol = 0.05; // RBS local connection resolution
	int RBS_max_depth = 150; // Maximum RBS recursion depth
	int n; // Dimension of system
};





#endif /* CHECKER_H_ */
