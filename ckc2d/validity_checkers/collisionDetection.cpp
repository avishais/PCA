#include "collisionDetection.h"


collisionDetection::collisionDetection() {

	n_ = NUM_LINKS;
	L_.resize(n_);
	for (int i = 0; i < n_; i++)
		L_[i] = LINK_LENGTH;
	qminmax_ = QMINMAX;
}

// Returns 0 if no collision
int collisionDetection::collision_state(State q) {
	if ( !check_angles(q) || !self_collision(q) || !obstacle_collision(q) )
		return 1;
	else 
		return 0;
}

double collisionDetection::normDistance(State a1, State a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}

double collisionDetection::normVector(State q) {

	double sum;
	for (int i = 0; i < n_; i++)
		sum += q[i]*q[i];

	return sqrt(sum);
}

// ------------------------------- Constraints functions ---------------------------

bool collisionDetection::check_angles(State q, double factor) {

	for (int i = 0; i < n_-1; i++)
		if (q[i] > factor*qminmax_ || q[i] < -factor*qminmax_)
			return false;
	if (q[n_-1] < 0)
		return false;

	return true;
}

bool collisionDetection::self_collision(State q, double factor) {
	double Ax, Ay, Bx, By, Cx, Cy, Dx, Dy;
	Ax = Ay = 0;

	double cum_q = 0, cum_q_CD;
	for (int i = 0; i < n_-3; i++) {
		cum_q += q[i];

		Bx = Ax + L_[i]*cos(cum_q);
		By = Ay + L_[i]*sin(cum_q);

		cum_q_CD = cum_q + q[i+1];
		Cx = Bx + L_[i]*cos(cum_q_CD);
		Cy = By + L_[i]*sin(cum_q_CD);
		for (int j = i+2; j < n_-1; j++) {
			cum_q_CD += q[j];
			Dx = Cx + L_[j]*cos(cum_q_CD);
			Dy = Cy + L_[j]*sin(cum_q_CD);

			// ---- Check if two lines intersect ---
			double s1_x = Bx - Ax;
			double s1_y = By - Ay;
			double s2_x = Dx - Cx;
			double s2_y = Dy - Cy;
			double s = (-s1_y * (Ax - Cx) + s1_x * (Ay - Cy)) / (-s2_x * s1_y + s1_x * s2_y);
			double t = ( s2_x * (Ay - Cy) - s2_y * (Ax - Cx)) / (-s2_x * s1_y + s1_x * s2_y);

			double minLim = -0.2 * factor, maxLim = 1.2 * factor;
			if (s >= minLim && s <= maxLim && t >= minLim && t <= maxLim)
				return false;
			// -------------------------------------

			Cx = Dx;
			Cy = Dy;
		}

		Ax = Bx;
		Ay = By;
	}

	return true;
}

// Returns false if the lines AB and CD intersect, otherwise true.
// Currently only checks when lines are not parallel
bool collisionDetection::LinesIntersect(State A, State B, State C, State D) {
	double s1_x, s1_y, s2_x, s2_y;
	s1_x = B[0] - A[0];
	s1_y = B[1] - A[1];
	s2_x = D[0] - C[0];
	s2_y = D[1] - C[1];

	double s, t;
	s = (-s1_y * (A[0] - C[0]) + s1_x * (A[1] - C[1])) / (-s2_x * s1_y + s1_x * s2_y);
	t = ( s2_x * (A[1] - C[1]) - s2_y * (A[0] - C[0])) / (-s2_x * s1_y + s1_x * s2_y);

	double minLim = -0.2, maxLim = 1.2;
	if (s >= minLim && s <= maxLim && t >= minLim && t <= maxLim)
		return false;

	return true; // No collision
}

bool collisionDetection::obstacle_collision(State q, double factor) {
	double x, y;
	x = y = 0;

	double cum_q = 0;
	for (int i = 0; i < n_-1; i++) {
		cum_q += q[i];

		x = x + L_[i]*cos(cum_q);
		y = y + L_[i]*sin(cum_q);

		for (int j = 0; j < obs.size(); j++) {
			if ( (x-obs[j][0])*(x-obs[j][0]) + (y-obs[j][1])*(y-obs[j][1]) < (obs[j][2]+factor*L_[i])*(obs[j][2]+factor*L_[i]) )
				return false;
		}
	}
	return true;
}

