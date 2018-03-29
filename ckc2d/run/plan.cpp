/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Avishai Sintov, Ioan Sucan */

#include "plan.h"

bool isStateValid(const ob::State *state)
{
	return true;
}

ob::PlannerPtr plan_C::allocatePlanner(ob::SpaceInformationPtr si, plannerType p_type)
{
	switch (p_type) {
   		// case PLANNER_BIRRT:
        // {
        //     return std::make_shared<og::CBiRRT>(si, dimension_, maxStep, dim_, knn_);
        //     break;
        // }
        case PLANNER_RRT:
        {
            return std::make_shared<og::RRT>(si, dimension_, maxStep, dim_, knn_);
            break;
        }
        // case PLANNER_LAZYRRT:
        // {
        //     return std::make_shared<og::LazyRRT>(si, dimension_, maxStep, dim_, knn_);
        //     break;
        // }
        // case PLANNER_SBL:
        // {
        //     return std::make_shared<og::SBL>(si, dimension_, maxStep, dim_, knn_);
        //     break;
        // }
		case PLANNER_RRTC:
        {
            return std::make_shared<og::RRTConnect>(si, dimension_, maxStep, dim_, knn_);
            break;
        }
	default:
	{
		OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
		return ob::PlannerPtr(); // Address compiler warning re: no return value.
		break;
	}
	}
}

void plan_C::plan(State c_start, State c_goal, double runtime, plannerType ptype, double max_step, int dim, int knn) {

	int n = c_start.size();

	// construct the state space we are planning inz
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(n)); // Angles of CKC - R^n

	// set the bounds for the Q=R^n part of 'Cspace'
	ob::RealVectorBounds Qbounds(n);
	for (int i = 0; i < n-1; i++) {
		Qbounds.setLow(i, -PI_);
		Qbounds.setHigh(i, PI_);
	}
	Qbounds.setLow(n-1, 0);
	Qbounds.setHigh(n-1, 2*PI_);

	// set the bound for the compound space
	Q->as<ob::RealVectorStateSpace>()->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(Q);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	// set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.02); // 3% ???

	// create start state
	ob::ScopedState<ob::RealVectorStateSpace> start(Cspace);
	for (int i = 0; i < n; i++) {
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = c_start[i]; // Access the first component of the start a-state
	}

	// create goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(Cspace);
	for (int i = 0; i < n; i++) {
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = c_goal[i]; // Access the first component of the goal a-state
	}

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

	// Properties
	maxStep = max_step;
	knn_ = knn;
	dim_ = dim;
	dimension_ = c_start.size();
	
	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner = allocatePlanner(si, ptype);

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	//planner->printSettings(std::cout); // Prints some parameters such as range
	//planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	// print the settings for this space
	//si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	//si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	// print the problem settings
	//pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	// attempt to solve the problem within one second of planning time
	auto begin = Clock::now();
	ob::PlannerStatus solved = planner->solve(runtime);
	cout << "Runtime: " << std::chrono::duration<double>(Clock::now() - begin).count() << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution:" << std::endl;

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		//std::ofstream myfile;
		//myfile.open("pathGD.txt");
		//og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		//pog.printAsMatrix(myfile); // Print as matrix to file
		//myfile.close();
		solved_bool = true;
	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}

}

int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime;
	plannerType ptype;
	string plannerName;

	if (argn == 1) {
		runtime = 1; // sec
		ptype = PLANNER_BIRRT;
		plannerName = "CBiRRT";
	}
	else if (argn == 2) {
		runtime = atof(args[1]);
		ptype = PLANNER_BIRRT;
		plannerName = "CBiRRT";
	}
	else if (argn > 2) {
		runtime = atof(args[1]);
		switch (atoi(args[2])) {
		case 1 :
			ptype = PLANNER_BIRRT;
			plannerName = "CBiRRT";
			break;
		case 2 :
			ptype = PLANNER_RRT;
			plannerName = "RRT";
			break;
		case 3 :
			ptype = PLANNER_LAZYRRT;
			plannerName = "LazyRRT";
			break;
		case 4 :
			ptype = PLANNER_SBL;
			plannerName = "SBL";
			break;
		case 5 :
			ptype = PLANNER_RRTC;
			plannerName = "RRTConnect";
			break;
		default :
			cout << "Error: Requested planner not defined.";
			exit(1);
		}
	}

	plan_C Plan;

	srand( time(NULL) );

	State c_start = {1.6581, 0.17453, 0.17453, 0.17453, -0.034907, -0.17453, -0.17453, -0.5236, -0.69813, -0.5236, -0.87266, -0.17453, 0.087266, 0.34907, 0.17453, 0.17453, 0.17453, 0.18147, -0.80904, 2.4791};
	State c_goal = {-2.1293, 0.34907, 0.5236, 0.5236, 0.69813, 0.61087, 0.61087, -0.17453, -0.7854, -0.5236, -0.34907, 0.5236, 0.7854, 0.7854, 0.2618, 0.43633, -0.17453, -1.2474, 1.2172, 5.0836}; // 4 obs


	int mode = 2;
	switch (mode) {
	case 1: {//Manual check
		Plan.plan(c_start, c_goal, runtime, ptype, 0.3, 0*(c_start.size()-3), 40);

		break;
	}
	case 2 : { // Benchmark method with constant d
		ofstream GD;
		double d = 0.3;
		GD.open("./matlab/Benchmark_" + plannerName + "_w_s03_d17_kR.txt", ios::app);

		for (int k = 0; k < 50; k++) {
			Plan.plan(c_start, c_goal, runtime, ptype, d, 17, 20); 

			// Extract from perf file
			ifstream FromFile;
			FromFile.open("./paths/perf_log.txt");
			string line;
			while (getline(FromFile, line))
				GD << line << "\t";
			FromFile.close();
			GD << endl;
		}
		GD.close();
		break;
	}

	}

	std::cout << std::endl << std::endl;

	return 0;
}

