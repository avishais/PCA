/*
 * plan_C_space.h
 *
 *  Created on: Nov 10, 2016
 *      Author: avishai
 */

#ifndef PLAN_C_SPACE_H_
#define PLAN_C_SPACE_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// Modified and custom planners
#include "../planners/CBiRRT.h"
#include "../planners/RRT.h"
// #include "../planners/LazyRRT.h"
// #include "../planners/PRM.h"
// #include "../planners/SBL.h"

//#include "../validity_checkers/verification_class.h"

// Standard libraries
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

// An enum of available planners
enum plannerType
{
	PLANNER_BIRRT,
	PLANNER_RRT,
	PLANNER_LAZYRRT,
	PLANNER_PRM,
	PLANNER_SBL
};

bool isStateValid(const ob::State *state);

// Prototypes
class plan_C
{
public:

	void plan(State, State, double, plannerType = PLANNER_BIRRT, double = 2, int = 6, int = 20);

	// Construct the planner specified by our command line argument.
	// This helper function is simply a switch statement.
	ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr, plannerType);

	bool solved_bool;
	double total_runtime;
	int ode_count;

	double maxStep;
	int knn_;
	int dim_;

	//verification_class vfc;

	int env;
	void set_environment(int env_index) {
		env = env_index;
	}
};

#endif /* PLAN_C_SPACE_H_ */
