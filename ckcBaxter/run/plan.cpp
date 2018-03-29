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

/* Author: Ioan Sucan, Avishai Sintov */

#include "plan.h"

bool isStateValid(const ob::State *state)
{
	return true;
}

ob::PlannerPtr plan_C::allocatePlanner(ob::SpaceInformationPtr si, plannerType p_type)
{
    switch (p_type)
    {
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
        case PLANNER_LAZYRRT:
        {
            return std::make_shared<og::LazyRRT>(si, dimension_, maxStep, dim_, knn_);
            break;
        }
        // case PLANNER_SBL:
        // {
        //     return std::make_shared<og::SBL>(si, maxStep, env);
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

	// construct the state space we are planning inz
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(14)); // Angles of Robot 1 & 2 - R^14

	// set the bounds for the Q=R^7 part of 'Cspace'
	ob::RealVectorBounds Qbounds(14);
	Qbounds.setLow(0, -2.461); // S0
	Qbounds.setHigh(0, 0.89);
	Qbounds.setLow(1, -2.147); // S1
	Qbounds.setHigh(1, +1.047);
	Qbounds.setLow(2, -3.028); // E0
	Qbounds.setHigh(2, +3.028);
	Qbounds.setLow(3, -1.6232); // E1
	Qbounds.setHigh(3, 1.0472);
	Qbounds.setLow(4, -3.059); // W0
	Qbounds.setHigh(4, 3.059);
	Qbounds.setLow(5, -1.571); // W1
	Qbounds.setHigh(5, 2.094);
	Qbounds.setLow(6, -3.059); // W2
	Qbounds.setHigh(6, 3.059);
	// Left arm
	Qbounds.setLow(7, -0.89); // S0
	Qbounds.setHigh(7, 2.462);
	Qbounds.setLow(8, -2.147); // S1
	Qbounds.setHigh(8, +1.047);
	Qbounds.setLow(9, -3.028); // E0
	Qbounds.setHigh(9, +3.028);
	Qbounds.setLow(10, -1.6232); // E1
	Qbounds.setHigh(10, 1.0472);
	Qbounds.setLow(11, -3.059); // W0
	Qbounds.setHigh(11, 3.059);
	Qbounds.setLow(12, -1.571); // W1
	Qbounds.setHigh(12, 2.094);
	Qbounds.setLow(13, -3.059); // W2
	Qbounds.setHigh(13, 3.059);

	// set the bound for the compound space
	Q->as<ob::RealVectorStateSpace>()->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(Q);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	// set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.1); // 3% ???

	// create start state
	ob::ScopedState<ob::RealVectorStateSpace> start(Cspace);
	for (int i = 0; i < 14; i++) {
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = c_start[i]; // Access the first component of the start a-state
	}

	// create goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(Cspace);
	for (int i = 0; i < 14; i++) {
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
	total_runtime =  std::chrono::duration<double>(Clock::now() - begin).count();
	cout << "Runtime: " << total_runtime << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//ob::PathPtr path = pdef->getSolutionPath();

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		//std::ofstream myfile;
		//myfile.open("pathGD.txt");
		//og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		//pog.printAsMatrix(myfile); // Print as matrix to file
		//myfile.close();

		std::cout << "Found solution:" << std::endl;
		solved_bool = true;
	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}
}

void extract_from_perf_file(ofstream &ToFile) {
	ifstream FromFile;
	FromFile.open("./paths/perf_log.txt");

	string line;
	while (getline(FromFile, line))
		ToFile << line << "\t";

	FromFile.close();
}

int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime; // Maximum allowed runtime
	plannerType ptype; // Planner type
	string plannerName;
	int env; // Tested environment index

	if (argn == 1) {
		runtime = 1; // sec
		ptype = PLANNER_BIRRT;
		plannerName = "CBiRRT";
		env = 1;
	}
	else if (argn == 2) {
		runtime = atof(args[1]);
		ptype = PLANNER_BIRRT;
		plannerName = "CBiRRT";
		env = 1;
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
		if (argn == 4)
			env = atoi(args[3]);
		else
			env = 1;
	}

	plan_C Plan;

	cout << "env: " << env << endl;

	srand (time(NULL));

	State c_start, c_goal;
	if (env == 1) {
		c_start = {0.463833, -0.83021, 0.823498, -1.17248, 0.985169, -0.483974, -0.811063, -0.506105, -1.259, 1.15372, -0.0200091, 0.311528, -0.679945, -2.7686 };
		c_goal = {-0.688322, -0.0425717, 1.93236, 0.700173, -0.990206, -1.32432, -1.39343, 0.23845, -0.0698612, -1.55172, -0.294552, -1.91595, 1.44318, 1.36148 };
		Plan.set_environment(1);
	}
	else if (env == 2) {
		c_start = {-1.09373, -1.68292, 0.99801, -1.48912, -1.01775, -0.431006, 2.47099, -0.101416, -0.33912, -2.96362, 0.10259, -1.87046, 0.665993, -2.84012 };
		c_goal = {0.589347, -1.08461, 0.399868, -0.915582, 1.23539, 1.86865, -0.714677, 0.271736, -0.267008, -1.42245, 0.24666, 0.16777, 2.04954, -0.749441}; 
		Plan.set_environment(2);
	}

	int mode = 2;
	switch (mode) {
	case 1: {

		// StateValidityChecker svc(14);
		// svc.initiate_log_parameters();

		// while (1) {
		// 	c_start = svc.sample_q();		
		// 	c_goal = svc.sample_q();

			Plan.plan(c_start, c_goal, runtime, ptype, 0.5, -1, 25);

		// 	if (Plan.solved_bool && Plan.total_runtime > 1)
		// 		break;
		// }

		break;
	}
	case 2 : { // Benchmark method with constant d
		ofstream GD;
		double d;
		if (env == 1) {
			GD.open("./matlab/Benchmark_" + plannerName + "_wo_s08.txt", ios::app);
			d = 0.8;
		}
		else if (env == 2) {
			GD.open("./matlab/Benchmark_" + plannerName + "_w_s14_d6_k40.txt", ios::app);
			d = 1.4;
		}

		for (int k = 0; k < 50; k++) {
			Plan.plan(c_start, c_goal, runtime, ptype, d, 6, 40); 

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
	case 3 : { // Benchmark maximum step size
		ofstream GD;
		if (env == 1)
			GD.open("./matlab/Benchmark_" + plannerName + "_wo_rB.txt", ios::app);
		else if (env == 2)
			GD.open("./matlab/Benchmark_" + plannerName + "_envII_w_rB.txt", ios::app);

		for (int k = 0; k < 100; k++) {

			for (int j = 0; j < 10; j++) {
				double maxStep = 0.2 + 0.2*j;

				cout << "** Running GD iteration " << k << " with maximum step: " << maxStep << " **" << endl;

				Plan.plan(c_start, c_goal, runtime, ptype, maxStep, -1);

				GD << maxStep << "\t";

				// Extract from perf file
				ifstream FromFile;
				FromFile.open("./paths/perf_log.txt");
				string line;
				while (getline(FromFile, line))
					GD << line << "\t";
				FromFile.close();
				GD << endl;
			}
		}
		GD.close();
		break;
	}
	case 4 : { // Benchmark min tree size
		ofstream GD;
		if (env == 1)
			GD.open("./matlab/Benchmark_" + plannerName + "_dimpca_knn.txt", ios::app);
		else if (env == 2)
			GD.open("./matlab/Benchmark_" + plannerName + "_dimpca_knn.txt", ios::app);

		for (int k = 0; k < 2000; k++) {

			//int knn = 60;
			for (int knn = 5; knn <= 120; knn+=20) {
				for (int j = 1; j < 15; j+=2) {
					int dim = 0 + 1 * j;
					double maxStep = 1;

					cout << "** Running GD iteration " << k << " with knn = " << knn << " and dim = " << dim << " **" << endl;

					Plan.plan(c_start, c_goal, runtime, ptype, maxStep, dim, knn);

					GD << dim << "\t" << knn << "\t";
					//GD << dim << "\t";

					// Extract from perf file
					ifstream FromFile;
					FromFile.open("./paths/perf_log.txt");
					string line;
					while (getline(FromFile, line))
						GD << line << "\t";
					FromFile.close();
					GD << endl;
				}
		}
		}
		GD.close();
		break;
	}
	}

	std::cout << std::endl << std::endl;

	return 0;
}

