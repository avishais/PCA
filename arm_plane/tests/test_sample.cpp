#include "../proj_classes/projectC.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double constraint(const column_vector &m);

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
	projectC pj(7);

	State q(7, 0);
	q = pj.random_q();
   
	while (!pj.project(q)) 
		q = pj.random_q();

	cout << "Sampled.\n";

	pj.get_constraint_val(q);

	pj.log_q(q);

	return 0;
}

