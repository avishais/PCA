#include "../validity_checkers/StateValidityCheckerGD.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

using namespace gd;

int main() {

	int Seed = time(NULL); //1502858101;//
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	int n = 4;

	// KDL
	StateValidityChecker svc(n, 0.3);
	
	State q(n);

	std::ofstream f;
	f.open("./results/samples2d_4.txt", ios::app);

	int N = 1e6, i = 0;
	while (i < N) {
		q = svc.sample_q();

		if (q[0] < -900)
		continue;

		svc.log_q(q, true);
		//cin.ignore();

		for (int j = 0; j < q.size(); j++)
			f << q[j] << " ";
		f << endl;

		i++;
		if (!(i % 100))
			cout << (double)i/N*100 << "%\n";
	}

	f.close();
	return 0;
}

