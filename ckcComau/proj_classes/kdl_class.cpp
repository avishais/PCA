#include "kdl_class.h"

// Constructor for the robots
kdl::kdl() {
	b = 344; // Height of base
	l1x = 524.5; 
	l1z = 199;
	l2 = 1250; 
	l3x = 186.5; 
	l3z = 210; 
	l4 = 1250; 
	l5 = 116.5; 
	lee = 22;

	initMatrix(Q, 4, 4);
	Q = {{1,0,0,ROD_LENGTH},{0,1,0,0},{0,0,1,0},{0,0,0,1}}; // Rod transformation from one end-tip to the other

	// Joint limits
	q1minmax = deg2rad(180);
	q2max = deg2rad(125);
	q2min = deg2rad(-60);
	q3max = deg2rad(75);
	q3min = deg2rad(-90);
	q4minmax = deg2rad(2700);
	q5minmax = deg2rad(120);
	q6minmax = deg2rad(400);

	initMatrix(T_fk, 4, 4);

	initMatrix(T_pose, 4, 4);
	T_pose = {{1, 0, 0, ROBOTS_DISTANCE_X}, {0, 1, 0, ROBOTS_DISTANCE_Y}, {0, 0, 1, ROBOT2_HEIGHT-ROBOT1_HEIGHT}, {0, 0, 0, 1}};

	//Definition of a kinematic chain & add segments to the chain
	// Robot 1
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0,0.0,b)))); // Base link
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(l1x,0.0,l1z)))); // Link 1
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,l2)))); // Link 2
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l3x,0.0,l3z)))); // Link 3
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l4,0.0,0.0)))); // Link 4
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l5,0.0,0.0)))); // Link 5
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(lee,0.0,0.0)))); // EE
	// Box
	Rotation r( Q[0][0],Q[0][1],Q[0][2], Q[1][0],Q[1][1],Q[1][2], Q[2][0],Q[2][1],Q[2][2] );
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(Q[0][3],Q[1][3],Q[2][3])))); // Box translation
	chain.addSegment(Segment(Joint(Joint::None),Frame(r))); // Box rotation
	// Robot 2 - backwards
	chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(lee,0.0,0.0)))); // Box translation	
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l5,0.0,0.0)))); // Link 5
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l4,0.0,0.0)))); // Link 4
	chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l3x,0.0,-l3z)))); // Link 3 "-"
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,-l2)))); // Link 2
	chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l1x,0.0,-l1z)))); // Link 1
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,-b)))); // Base link

	// Create joint array
	unsigned int nj = chain.getNrOfJoints();
	jointpositions = JntArray(nj);

	// Joint limits for KDL
	q_min = JntArray(nj);
	q_max = JntArray(nj);
	State q_min_vec = {-q1minmax, q2min, q3min, -q4minmax, -q5minmax, -q6minmax, -q6minmax, -q5minmax, -q4minmax, q3min, q2min, -q1minmax};
	State q_max_vec = { q1minmax, q2max, q3max,  q4minmax,  q5minmax,  q6minmax,  q6minmax,  q5minmax,  q4minmax, q3max, q2max,  q1minmax};
	for (int i = 0; i < nj; i++) {
		q_min(i) = q_min_vec[i];
		q_max(i) = q_max_vec[i];
	}

	initVector(q_solution, nj);

	cout << "Initiated chain with " << nj << " joints.\n";
}

// ----- Descend -------

bool kdl::GD(State q_init_flip) {

	bool valid = true;

	// Flip robot two vector
	State q_init(q_init_flip.size());
	for (int i = 0; i < 6; i++)
		q_init[i] = q_init_flip[i];
	for (int i = 11, j = 6; i >= 6; i--,j++)
		q_init[j] = q_init_flip[i];
	q_init[11] = -q_init[11];

	State q(12);

	IK_counter++;
	clock_t begin = clock();

	for (int i = 0; i < 3; i++) {
		cartposIK.p(i) = T_pose[i][3];
		for (int j = 0; j < 3; j++)
			cartposIK.M(i,j) = T_pose[i][j];
	}

	// KDL
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain); 	// Create solver based on kinematic chain
	ChainIkSolverVel_pinv iksolverv(chain);//Inverse velocity solver
	ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,10000,1e-5);//Maximum 10000 iterations, stop at accuracy 1e-5

	//Creation of jntarrays:
	JntArray qKDL(chain.getNrOfJoints());
	JntArray qInit(chain.getNrOfJoints());

	for (int i = 0; i < chain.getNrOfJoints(); i++)
		qInit(i) = q_init[i];

	//Set destination frame
	KDL::Frame F_dest = cartposIK;//Frame(Vector(1.0, 1.0, 0.0));
	int ret = iksolver.CartToJnt(qInit, F_dest, qKDL);

	bool result = false;
	if (ret >= 0) {

		for (int i = 0; i < 12; i++)
			if (fabs(qKDL(i)) < 1e-4)
				q[i] = 0;
			else
				q[i] = qKDL(i);

		for (int i = 0; i < q.size(); i++) {
			q[i] = fmod(q[i], 2*PI);
			if (q[i]>PI)
				q[i] -= 2*PI;
			if (q[i]<-PI)
				q[i] += 2*PI;
		}

		for (int i = 0; i < 6; i++)
			q_solution[i] = q[i];
		for (int i = 11, j = 6; i >= 6; i--,j++)
			q_solution[j] = q[i];
		q_solution[6] = -q_solution[6];

		if (include_joint_limits && (!check_angle_limits(q_solution) || !checkEE(q_solution)))
			result = false;
		else
			result = true;

		proj_dist += norm(q_solution, q_init_flip);
	}

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	return result;
}

bool kdl::GD_JL(State q_init_flip) {

	bool valid = true;

	// Flip robot two vector
	State q_init(q_init_flip.size());
	for (int i = 0; i < 6; i++)
		q_init[i] = q_init_flip[i];
	for (int i = 11, j = 6; i >= 6; i--,j++)
		q_init[j] = q_init_flip[i];
	q_init[11] = -q_init[11];

	State q(12);

	IK_counter++;
	clock_t begin = clock();

	for (int i = 0; i < 3; i++) {
		cartposIK.p(i) = T_pose[i][3];
		for (int j = 0; j < 3; j++)
			cartposIK.M(i,j) = T_pose[i][j];
	}

	// KDL
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain); 	// Create solver based on kinematic chain
	ChainIkSolverVel_pinv iksolverv(chain);//Inverse velocity solver
	ChainIkSolverPos_NR_JL iksolver(chain, q_min, q_max, fksolver,iksolverv,10000,1e-5);//Maximum 10000 iterations, stop at accuracy 1e-5

	//Creation of jntarrays:
	JntArray qKDL(chain.getNrOfJoints());
	JntArray qInit(chain.getNrOfJoints());

	for (int i = 0; i < chain.getNrOfJoints(); i++)
		qInit(i) = q_init[i];

	//Set destination frame
	KDL::Frame F_dest = cartposIK;//Frame(Vector(1.0, 1.0, 0.0));
	int ret = iksolver.CartToJnt(qInit, F_dest, qKDL);

	bool result = false;
	if (ret >= 0) {

		for (int i = 0; i < 12; i++)
			if (fabs(qKDL(i)) < 1e-4)
				q[i] = 0;
			else
				q[i] = qKDL(i);

		for (int i = 0; i < q.size(); i++) {
			q[i] = fmod(q[i], 2*PI);
			if (q[i]>PI)
				q[i] -= 2*PI;
			if (q[i]<-PI)
				q[i] += 2*PI;
		}

		for (int i = 0; i < 6; i++)
			q_solution[i] = q[i];
		for (int i = 11, j = 6; i >= 6; i--,j++)
			q_solution[j] = q[i];
		q_solution[6] = -q_solution[6];

		result = true;
	}

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	return result;
}

bool kdl::check_angle_limits(State q) {

	if (fabs(q[0]) > q1minmax)
		return false;
	if (q[1] < q2min || q[1] > q2max)
		return false;
	if (q[2] < q3min || q[2] > q3max)
		return false;
	if (fabs(q[3]) > q4minmax)
		return false;
	if (fabs(q[4]) > q5minmax)
		return false;
	if (fabs(q[5]) > q6minmax)
		return false;

	if (fabs(q[6]) > q1minmax)
		return false;
	if (q[7] < q2min || q[7] > q2max)
		return false;
	if (q[8] < q3min || q[8] > q3max)
		return false;
	if (fabs(q[9]) > q4minmax)
		return false;
	if (fabs(q[10]) > q5minmax)
		return false;
	if (fabs(q[11]) > q6minmax)
		return false;

	return true;
}

State kdl::get_GD_result() {
	return q_solution;
}

// -----FK-------

// This is only for validation. There is no use for this function in terms of closed chain kinematics
void kdl::FK(State q) {

	// Flip robot two vector
	State q_flip(q.size());
	for (int i = 0; i < 6; i++)
		q_flip[i] = q[i];
	for (int i = 11, j = 6; i >= 6; i--,j++)
		q_flip[j] = q[i];
	q_flip[11] = -q_flip[11];

	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

	for (int i = 0; i < q_flip.size(); i++)
		jointpositions(i) = q_flip[i];

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions, cartposFK);

	for (int i = 0; i < 3; i++) {
		T_fk[i][3] = cartposFK.p(i);
		for (int j = 0; j < 3; j++) {
			T_fk[i][j] = cartposFK.M(i,j);
			T_fk[i][j] = fabs(T_fk[i][j]) < 1e-4 ? 0 : T_fk[i][j];
		}
	}
	T_fk[3][0] = T_fk[3][1] = T_fk[3][2] = 0;
	T_fk[3][3] = 1;
}

// This is only for validation. There is no use for this function in terms of closed chain kinematics
void kdl::FK6(State q) {

	Chain chain6;
	chain6.addSegment(Segment(Joint(Joint::None),Frame(Vector(0.0,0.0,b)))); // Base link
	chain6.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(l1x,0.0,l1z)))); // Link 1
	chain6.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(0.0,0.0,l2)))); // Link 2
	chain6.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l3x,0.0,l3z)))); // Link 3
	chain6.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(l4,0.0,0.0)))); // Link 4
	chain6.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(l5,0.0,0.0)))); // Link 5
	chain6.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(lee,0.0,0.0)))); // EE

	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain6);

	KDL::JntArray jointpositions6 = JntArray(6);
	KDL::Frame cartposFK6;

	for (int i = 0; i < q.size(); i++)
		jointpositions6(i) = q[i];

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions6, cartposFK6);

	for (int i = 0; i < 3; i++) {
		T_fk[i][3] = cartposFK6.p(i);
		for (int j = 0; j < 3; j++) {
			T_fk[i][j] = cartposFK6.M(i,j);
			T_fk[i][j] = fabs(T_fk[i][j]) < 1e-4 ? 0 : T_fk[i][j];
		}
	}
	T_fk[3][0] = T_fk[3][1] = T_fk[3][2] = 0;
	T_fk[3][3] = 1;
}

Matrix kdl::get_FK_solution() {

	//printMatrix(T_fk_solution_1);

	return T_fk;
}

// --------------------------- Check EE above ground -----------------------

bool kdl::checkEE(State q) {

	State q1(6), q2(6);
	for (int i = 0; i < 6; i++) {
		q1[i] = q[i];
		q2[i] = q[i+6];
	}

	FK6(q1);
	Matrix T1 = get_FK_solution();

	FK6(q2);
	Matrix T2 = get_FK_solution();

	if (T1[2][3] < 10 || T2[2][3] < 10)
		return false;

	return true;
}

//------------------------

// Misc
void kdl::initMatrix(Matrix &M, int n, int m) {
	M.resize(n);
	for (int i = 0; i < n; ++i)
		M[i].resize(m);
}

void kdl::initVector(State &V, int n) {
	V.resize(n);
}

double kdl::deg2rad(double deg) {
	return deg * PI / 180.0;
}

void kdl::printMatrix(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

void kdl::printVector(State p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++)
		cout << p[i] << " ";
	cout << "]" << endl;
}


void kdl::clearMatrix(Matrix &M) {
	for (unsigned i=0; i<M.size(); ++i)
		for (unsigned j=0; j<M[i].size(); ++j)
			M[i][j] = 0;;
}

void kdl::log_q(State q) {
	std::ofstream myfile;
	myfile.open("../paths/path.txt");

	myfile << 1 << endl;

	for (int i = 0; i < q.size(); i++)
		myfile << q[i] << " ";
	myfile << endl;

	myfile.close();
}

double kdl::norm(State q1, State q2) {

    double d = 0;
	for (int i = 0; i < q1.size(); i++)
        d += (q1[i] - q2[i]) * (q1[i] - q2[i]);
    
    return sqrt(d);
}
