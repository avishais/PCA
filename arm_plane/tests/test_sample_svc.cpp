#include "../validity_checkers/StateValidityChecker.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double constraint(const column_vector &m);

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
    StateValidityChecker pj;
    pj.initiate_log_parameters();

	State q = pj.sample_q();

	cout << "Sampled.\n";

	pj.get_constraint_val(q);

	pj.log_q(q);

	return 0;
}

