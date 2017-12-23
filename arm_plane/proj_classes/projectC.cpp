
#include "projectC.h"

// ========================================================================================================================

double height(State q) {

    double Z = 10*cos(q[4])*(sin(q[1])*(cos(q[3])) - cos(q[1])*cos(q[2])*( - sin(q[3]))) - sin(q[1])*( - 100*sin(q[3])) - (2707*sin(q[1])*( - sin(q[3])))/10 - (17471*sin(q[1]))/50 - 310*cos(q[5])*(sin(q[1])*(- sin(q[3])) + cos(q[1])*cos(q[2])*(cos(q[3]))) + 310*sin(q[5])*(cos(q[4])*(sin(q[1])*(cos(q[3])) - cos(q[1])*cos(q[2])*(- sin(q[3]))) + cos(q[1])*sin(q[2])*sin(q[4])) - 69*cos(q[1])*cos(q[2]) + 10*cos(q[1])*sin(q[2])*sin(q[4]) - (2707*cos(q[1])*cos(q[2])*(cos(q[3])))/10 - cos(q[1])*cos(q[2])*(100*cos(q[3])) + 374.6260;
    
    return Z;
}

double pitch(State q) {

    double r31 = sin(q[5])*(cos(q[4])*(sin(q[1])*(cos(q[3]) + 0) - cos(q[1])*cos(q[2])*(0 - sin(q[3]))) + cos(q[1])*sin(q[2])*sin(q[4])) - cos(q[5])*(sin(q[1])*(0 - sin(q[3])) + cos(q[1])*cos(q[2])*(cos(q[3]) + 0));
    
    double r32 = - sin(q[6])*(cos(q[5])*(cos(q[4])*(sin(q[1])*(cos(q[3]) + 0) - cos(q[1])*cos(q[2])*(0 - sin(q[3]))) + cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(sin(q[1])*(0 - sin(q[3])) + cos(q[1])*cos(q[2])*(cos(q[3]) + 0))) - cos(q[6])*(sin(q[4])*(sin(q[1])*(cos(q[3]) + 0) - cos(q[1])*cos(q[2])*(0 - sin(q[3]))) - cos(q[1])*cos(q[4])*sin(q[2]));    

    double r33 = sin(q[6])*(sin(q[4])*(sin(q[1])*(cos(q[3]) + 0) - cos(q[1])*cos(q[2])*(0 - sin(q[3]))) - cos(q[1])*cos(q[4])*sin(q[2])) - cos(q[6])*(cos(q[5])*(cos(q[4])*(sin(q[1])*(cos(q[3]) + 0) - cos(q[1])*cos(q[2])*(0 - sin(q[3]))) + cos(q[1])*sin(q[2])*sin(q[4])) + sin(q[5])*(sin(q[1])*(0 - sin(q[3])) + cos(q[1])*cos(q[2])*(cos(q[3]) + 0)));    
    
    if (r31 == 0)
        r31 = 0;
    if (r32 == 0) 
        r32 = 0;
    if (r33 == 0) 
        r33 = 0;

    //cout << r31 << " " << r32 << " " << r33 << endl;

    double p = atan2(-r31, sqrt(r32*r32 + r33*r33));
       
    return p;
}

double constraint(const column_vector &m) {

    State q(7);
    for (int i = 0; i < q.size(); i++) 
        q[i] = m(i);
   
    return (height_d - height(q)) * (height_d - height(q)) + (pitch_d - pitch(q)) * (pitch_d - pitch(q));
}

// ========================================================================================================================

projectC::projectC(int n) : solver(LBFGS), n_(n) {

    	// Joint limits
	qmin.resize(7);
	qmax.resize(7);
	qmin[0] = deg2rad(-141); // S0 - Flip for Arm 2
	qmax[0] = deg2rad(51);
	qmin[1] = deg2rad(-123); // S1
	qmax[1] = deg2rad(60);
	qmin[2] = deg2rad(-173.5); // E0 - Flip for Arm 2
	qmax[2] = deg2rad(173.5);
	qmin[3] = deg2rad(-93); // E1
	qmax[3] = deg2rad(60);
	qmin[4] = deg2rad(-175.25); // W0 - Flip for Arm 2
	qmax[4] = deg2rad(175.25);
	qmin[5] = deg2rad(-90); // W1
	qmax[5] = deg2rad(120);
	qmin[6] = deg2rad(-175.25); // W2 - Flip for Arm 2
    qmax[6] = deg2rad(175.25);
};

bool projectC::project(State &q_init) {

    IK_counter++;
    clock_t begin = clock();
    
    column_vector starting_point(n_);

    State q_prev = q_init;
    
    for (int i = 0; i < n_; i++) 
        starting_point(i) = q_init[i];

    if (solver == BFGS)
    find_min_box_constrained(bfgs_search_strategy(),
            objective_delta_stop_strategy(1e-4),
            constraint, derivative(constraint), starting_point, -PI, PI);
    else
    find_min_box_constrained(lbfgs_search_strategy(10),
            objective_delta_stop_strategy(1e-4),
            constraint, derivative(constraint), starting_point, -PI, PI);

    // if (solver == BFGS)
    //     find_min_using_approximate_derivatives(bfgs_search_strategy(),
    //         objective_delta_stop_strategy(1e-4),
    //         constraint, starting_point, -1);
    // else
    //     find_min_using_approximate_derivatives(lbfgs_search_strategy(10),
    //         objective_delta_stop_strategy(1e-4),
    //         constraint, starting_point, -1);
    
    for (int i = 0; i < n_; i++) {
        q_init[i] = starting_point(i);

        // if (fabs(q_init[i]) < 1e-4)
        //     q_init[i] = 0;
        q_init[i] = fmod(q_init[i], 2 * PI);
        if (q_init[i] > PI)
            q_init[i] -= 2 * PI;
        if (q_init[i] < -PI)
            q_init[i] += 2 * PI;
    }

    bool result = true;
    if (fabs(height(q_init)-height_d) > 3 || fabs(pitch(q_init)-pitch_d) > 0.1 || !check_angle_limits(q_init) || !EE_bounds(q_init)) 
        result = false;

    IK_time += double(clock() - begin) / CLOCKS_PER_SEC;
    
    proj_dist += norm(q_prev, q_init);

    return result;
}

State projectC::random_q() {

    State q(n_);
    for (int i = 0; i < n_; i++) 
        q[i] = (double)std::rand() / RAND_MAX * 2*PI - PI;

    return q;
}

// void projectC::log_q(State q) {

//     std::ofstream myfile;

// 	myfile.open("../paths/path.txt");
// 	myfile << 1 << endl;

// 	for (int j = 0; j < 7; j++)
//         myfile << q[j] << " ";
        
//     for (int j = 0; j < 7; j++)
// 		myfile << 0 << " ";
// 	myfile << endl;

// 	myfile.close();
// }

double projectC::get_constraint_val(State q) {

    column_vector point(q.size());

    for (int i = 0; i < q.size(); i++)
        point(i) = q[i];

    cout << "Constraint value: " << constraint(point) << ", height: " << height(q) << ", pitch: " << pitch(q) << endl;

    return constraint(point);
}

bool projectC::check_angle_limits(State q) {

	// Arm 1
	for (int i = 0; i < n_; i++)
		if (q[i] < qmin[i] || q[i] > qmax[i])
            return false;

	return true;
}

bool projectC::EE_bounds(State q) {

    double x = 73*cos(q[0]) - (2707*(sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1]))*(cos(q[3]) + 0))/10 - (sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1]))*(100*cos(q[3]) + 0) - 310*sin(q[5])*(cos(q[4])*((sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1]))*(0 - sin(q[3])) + cos(q[0])*cos(q[1])*(cos(q[3]) + 0)) + sin(q[4])*(cos(q[2])*sin(q[0]) - cos(q[0])*sin(q[1])*sin(q[2]))) - 10*cos(q[4])*((sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1]))*(0 - sin(q[3])) + cos(q[0])*cos(q[1])*(cos(q[3]) + 0)) - 310*cos(q[5])*((sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1]))*(cos(q[3]) + 0) - cos(q[0])*cos(q[1])*(0 - sin(q[3]))) + (17471*cos(q[0])*cos(q[1]))/50 - 69*sin(q[0])*sin(q[2]) - 10*sin(q[4])*(cos(q[2])*sin(q[0]) - cos(q[0])*sin(q[1])*sin(q[2])) - 69*cos(q[0])*cos(q[2])*sin(q[1]) + cos(q[0])*cos(q[1])*(0 - 100*sin(q[3])) + (2707*cos(q[0])*cos(q[1])*(0 - sin(q[3])))/10 + 3017/50;
    double y = 73*sin(q[0]) + (2707*(cos(q[0])*sin(q[2]) - cos(q[2])*sin(q[0])*sin(q[1]))*(cos(q[3]) + 0))/10 + (cos(q[0])*sin(q[2]) - cos(q[2])*sin(q[0])*sin(q[1]))*(100*cos(q[3]) + 0) + 310*sin(q[5])*(cos(q[4])*((cos(q[0])*sin(q[2]) - cos(q[2])*sin(q[0])*sin(q[1]))*(0 - sin(q[3])) - cos(q[1])*sin(q[0])*(cos(q[3]) + 0)) + sin(q[4])*(cos(q[0])*cos(q[2]) + sin(q[0])*sin(q[1])*sin(q[2]))) + 10*cos(q[4])*((cos(q[0])*sin(q[2]) - cos(q[2])*sin(q[0])*sin(q[1]))*(0 - sin(q[3])) - cos(q[1])*sin(q[0])*(cos(q[3]) + 0)) + 310*cos(q[5])*((cos(q[0])*sin(q[2]) - cos(q[2])*sin(q[0])*sin(q[1]))*(cos(q[3]) + 0) + cos(q[1])*sin(q[0])*(0 - sin(q[3]))) + (17471*cos(q[1])*sin(q[0]))/50 + 69*cos(q[0])*sin(q[2]) + 10*sin(q[4])*(cos(q[0])*cos(q[2]) + sin(q[0])*sin(q[1])*sin(q[2])) - 69*cos(q[2])*sin(q[0])*sin(q[1]) + cos(q[1])*sin(q[0])*(0 - 100*sin(q[3])) + (2707*cos(q[1])*sin(q[0])*(0 - sin(q[3])))/10 - 250;

    //cout << x << " " << y << endl;

    if (x < x_min || x > x_max || y < y_min || y > y_max)
        return false;

	return true;
}


// -----------------------------------------------------------------------------------------


double projectC::deg2rad(double deg) {
	return deg * PI / 180.0;
}


void projectC::printMatrix(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

void projectC::printVector(State p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++)
		cout << p[i] << " ";
	cout << "]" << endl;
}

void projectC::printCVector(column_vector p) {
	cout << "{";
	for (unsigned i = 0; i < p.size(); i++)
		cout << p(i) << " ";
	cout << "}" << endl;
}

double projectC::norm(State q1, State q2) {

    double d = 0;
	for (int i = 0; i < q1.size(); i++)
        d += (q1[i] - q2[i]) * (q1[i] - q2[i]);
    
    return sqrt(d);
}
