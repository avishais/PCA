
#include <dlib/optimization.h>
#include <vector>
#include <string>
#include <iostream>
#include <math.h>

using namespace std;
using namespace dlib;

typedef std::vector<double> State;
typedef std::vector<std::vector< double >> Matrix;
typedef matrix<double, 0, 1> column_vector;

#define PI 3.14

const double height_d = 0, pitch_d = PI/2;

enum solver_type {BFGS, LBFGS};

class projectC {

public:
    projectC(int = 12);

    //double constraint(const column_vector &m);

    bool project(State&);

    State random_q();

    // void log_q(State);

    double get_constraint_val(State);

    /** Check the angles limits of the Baxter - IK based on a constant trans. matrix */
    bool check_angle_limits(State);

    /** Check if EE is in the workspace bounds */
    bool EE_bounds(State);
    
    double deg2rad(double);

    void printMatrix(Matrix);

    void printVector(State);
    
    void printCVector(column_vector);
    
    	/** Performance parameters */
	int IK_counter;
	double IK_time;
	int get_IK_counter() {
		return IK_counter;
	}
	double get_IK_time() {
		return IK_time;
    }
    
    void set_n(int n) {
        n_ = n;
    }

private:
    int n_;
    solver_type solver;

    State qmin, qmax; // Joint limits

    const double x_max = 1163, x_min = 147, y_max = 960, y_min = -960;
};