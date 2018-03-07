/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

// #include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "RRTConnect.h"

ompl::geometric::RRTConnect::RRTConnect(const base::SpaceInformationPtr &si, int dimension, double maxStep, int dimpca, int knn) : base::Planner(si, "RRTConnect"), StateValidityChecker(si, dimension)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    maxDistance_ = 0.0;

    Planner::declareParam<double>("range", this, &RRTConnect::setRange, &RRTConnect::getRange, "0.:1.:10000.");
    connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);

    defaultSettings();

    Range = maxStep;
    knn_ = knn;

    usePCA = dimpca < 1 || dimpca > dimension ? false : true;    
    dim_ = dimpca;
    nn_radius_ = 0.5;

	if (usePCA)
        cout << "*** Initiated PCA planning with with: knn = " << knn_ << ", and dim_pca = " << dim_ << " ***" << endl;
}

ompl::geometric::RRTConnect::~RRTConnect()
{
    freeMemory();
}

void ompl::geometric::RRTConnect::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    tStart_->setDistanceFunction(std::bind(&RRTConnect::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
    tGoal_->setDistanceFunction(std::bind(&RRTConnect::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
}

void ompl::geometric::RRTConnect::freeMemory()
{
    std::vector<Motion*> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::RRTConnect::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);
}

ompl::geometric::RRTConnect::GrowState ompl::geometric::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *nmotion, Motion *rmotion, int mode)
// mode = 1 -> extend, mode = 2 -> connect.
{
    State q(get_n());
    growTree_reached = false;

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
        dstate = tgi.xstate;
        reach = false;
    }

    // If trying to reach a point that does not satisfy the closure constraint - needs to be projected
    if (mode==1 || !reach) { // equivalent to (!(mode==2 && reach))

        // Project dstate (which currently is not on the manifold)
        clock_t sT = clock();
        if (!IKproject(dstate)) {  // Collision check is done inside the projection
            sampling_time += double(clock() - sT) / CLOCKS_PER_SEC;
            sampling_counter[1]++;
            return TRAPPED;
        }
        sampling_time += double(clock() - sT) / CLOCKS_PER_SEC;
        sampling_counter[0]++;

        si_->copyState(tgi.xstate, dstate);
        dstate = tgi.xstate;
    }

    // Check motion
    clock_t sT = clock();
    local_connection_count++;
    //bool validMotion = checkMotion(nmotion->state, dstate);
    bool validMotion = checkMotionRBS(nmotion->state, dstate);
    local_connection_time += double(clock() - sT) / CLOCKS_PER_SEC;

    if (validMotion)
    {
        local_connection_success_count++;

        /* create a motion */
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tgi.xmotion = motion;

        tree->add(motion);
        if (reach)
            return REACHED;
        else
            return ADVANCED;
    }
    else
        return TRAPPED;
}

ompl::base::PlannerStatus ompl::geometric::RRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
    initiate_log_parameters();
	base::State *start_node = si_->allocState();
	setRange(Range); // Maximum local connection distance *** will need to profile this value

	State q(get_n());

    checkValidity();
    startTime = clock();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);

        si_->copyState(start_node,st);        
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion   *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool startTree      = true;
    bool solved         = false;

    // Motion   *rmotion   = new Motion(si_);
	// rmotion->state = si_->allocState();

    while (ptc == false)
    {
        TreeData &tree      = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
            {
                Motion *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);

                PlanDistance = si_->distance(start_node, st);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = tree->nearest(rmotion);

        if (usePCA)// && tree->size() > 3)
            samplePCA(tree, nmotion, rstate);
        // if (usePCA)
        //     sampleProxPCA(nmotion, rstate);

        GrowState gs = growTree(tree, tgi, nmotion, rmotion, 1);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need top copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree;
            while (gsc == ADVANCED) {
                nmotion = otherTree->nearest(rmotion);                
                gsc = growTree(otherTree, tgi, nmotion, rmotion, 2);
            }

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion  = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // Report computation time
                total_runtime = double(clock() - startTime) / CLOCKS_PER_SEC;
                cout << "Solved in " << total_runtime << "s." << endl;
            
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion*> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion*> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                cout << "Path from tree 1 size: " << mpath1.size() << ", path from tree 2 size: " << mpath2.size() << endl;
                nodes_in_path = mpath1.size() + mpath2.size();
                nodes_in_trees = tStart_->size() + tGoal_->size();

                PathGeometric *path = new PathGeometric(si_);
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
                    path->append(mpath1[i]->state);
                for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
                    path->append(mpath2[i]->state);

                final_solved = true;
                LogPerf2file(); // Log planning parameters
                save2file(mpath1, mpath2);

                pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
                solved = true;
                break;
            }
        }
    }

    if (!solved)
	{
		// Report computation time
		endTime = clock();
		total_runtime = double(endTime - startTime) / CLOCKS_PER_SEC;

		nodes_in_trees = tStart_->size() + tGoal_->size();
		final_solved = false;
		LogPerf2file(); // Log planning parameters
	}

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::RRTConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (tStart_)
        tStart_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1),
                         base::PlannerDataVertex(motions[i]->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motions[i]->state, 2),
                         base::PlannerDataVertex(motions[i]->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}

void ompl::geometric::RRTConnect::samplePCA(TreeData &tree, Motion *nmotion, base::State *rstate) {

    std::vector<Motion*> nhbr;
    State q(get_n());

    // Find up to knn nearest neighbors to nmotion in the tree
    tree->nearestK(nmotion, std::min(knn_, (int)tree->size()), nhbr); //
    //tree->nearestR(nmotion, nn_radius_, nhbr);
    //if (nhbr.size() < 5)
      //  return;

    // Create vector<vector> db for neighbors
    Matrix NHBR;
    for (int i = 0; i < nhbr.size(); i++) {
        retrieveStateVector(nhbr[i]->state, q);
        NHBR.push_back(q);
    }
    retrieveStateVector(nmotion->state, q);
	NHBR.push_back(q);
	
	//retrieveStateVector(rstate, q);

    // Find new sample using pca
    q = sample_pca(NHBR, dim_);
    updateStateVector(rstate, q);
}  

void ompl::geometric::RRTConnect::sampleProxPCA(Motion *nmotion, base::State *rstate) {

    double proxMaxDistance = 0.02;
    base::State *Xstate = si_->allocState();
    State q(get_n());

    Matrix NHBR;
    int i = 0;
    while (i < knn_) {
        sampler_->sampleUniform(rstate);
        double d = si_->distance(nmotion->state, rstate);
        if (d > proxMaxDistance)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, proxMaxDistance / d, Xstate);
        }
        if (!IKproject(Xstate, false))
            continue;

        retrieveStateVector(Xstate, q);
        NHBR.push_back(q);
        i++;
    }
    retrieveStateVector(nmotion->state, q);
    NHBR.push_back(q);

    // Find new sample using pca
    q = sample_pca(NHBR, dim_);
    updateStateVector(rstate, q);
}


void ompl::geometric::RRTConnect::smoothPath(std::vector<Motion*> &path) {

	int orgSize = path.size();
	int s, c = 30;
	do {
		s = path.size();
		int i1, i2;
		do {
			i1 = std::rand() % path.size();
			i2 = std::rand() % path.size();
		} while (abs(i1-i2) < 2);

		if (checkMotionRBS(path[i1]->state, path[i2]->state))
			path.erase(path.begin()+min(i1,i2)+1, path.begin()+max(i1,i2));

		if (path.size() == s)
			c--;
		else
			c = 30;
	} while (c > 0);

	cout << "Reduce path size from " << orgSize << " milestones to " << path.size() << " milestones." << endl;

	nodes_in_path = path.size();
}


void ompl::geometric::RRTConnect::save2file(std::vector<Motion*> mpath1, std::vector<Motion*> mpath2) {

	cout << "Logging path to files..." << endl;

	int n = get_n();
	State q(n);

	{ // Log only milestones

		// Open a_path file
		std::ofstream myfile;
		myfile.open("./paths/path_milestones.txt");

		myfile << mpath1.size() + mpath2.size() << endl;

		for (int i = mpath1.size() - 1 ; i >= 0 ; --i) {
			retrieveStateVector(mpath1[i]->state, q);
			for (int j = 0; j<n; j++) {
				myfile << q[j] << " ";
			}
			myfile << endl;
		}
		for (unsigned int i = 0 ; i < mpath2.size() ; ++i) {
			retrieveStateVector(mpath2[i]->state, q);
			for (int j = 0; j<n; j++) {
				myfile << q[j] << " ";
			}
			myfile << endl;
		}

		myfile.close();
	}

	{ // Reconstruct RBS

		// Open a_path file
		std::ofstream fp, myfile;
		std::ifstream myfile1;
		myfile.open("./paths/temp.txt",ios::out);

		//myfile << mpath1.size() + mpath2.size() << endl;

		std::vector<Motion*> path;

		// Bulid basic path
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
			path.push_back(mpath1[i]);
		for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
			path.push_back(mpath2[i]);

		//smoothPath(path);

		retrieveStateVector(path[0]->state, q);
		for (int j = 0; j < q.size(); j++) 
			myfile << q[j] << " ";
		myfile << endl;

		int count = 1;
		for (int i = 1; i < path.size(); i++) {

			Matrix M;
			bool valid = false;
			valid =  reconstructRBS(path[i-1]->state, path[i]->state, M);

			if (!valid) {
				cout << "Error in reconstructing...\n";
				return;
			}

			for (int k = 1; k < M.size(); k++) {
				//pathLength += normDistance(M[k], M[k-1]);
				for (int j = 0; j < M[k].size(); j++) 
					myfile << M[k][j] << " ";
				myfile << endl;
				count++;
			}
		}

		// Update file with number of conf.
		myfile.close();
		myfile1.open("./paths/temp.txt",ios::in);
		fp.open("./paths/path.txt",ios::out);
		fp << count << endl;
		std::string line;
		while(myfile1.good()) {
			std::getline(myfile1, line ,'\n');
			fp << line << endl;
		}
		myfile1.close();
		fp.close();
		std::remove("./paths/temp.txt");

		//timeMinPath(path);
	}
}