#include "../validity_checkers/StateValidityChecker.h"
#include "kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
    StateValidityChecker svc;
    svc.initiate_log_parameters();

	State q(14,0);// = {0.463833, -0.83021, 0.823498, -1.17248, 0.985169, -0.483974, -0.811063, -0.506105, -1.259, 1.15372, -0.0200091, 0.311528, -0.679945, -2.7686 };
	//svc.sample_q();
	cout << svc.IKproject(q, false) << endl;
	
	svc.printVector(q);

	cout << "Sampled.\n";

	{
		kdl k;
		State q(14, 0);
		cout << k.GD(q) << endl;
		k.printVector(k.get_GD_result());
	}

	// svc.log_q(q);

	return 0;
}

