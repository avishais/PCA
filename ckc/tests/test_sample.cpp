#include "kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

bool withObs = false;

int main() {

	int Seed = time(NULL);
	srand(Seed);
	cout << "Seed in testing: " << Seed << endl;

	std::ofstream File;
	File.open("abb_ckc_samples.txt", ios::app);

	// KDL
	kdl pj;

	int N = 2e6-1e5;
	for (int i = 0; i < N; i++) {

		State q(12);
		while (1) {
			q = pj.rand_q();

			if (!pj.GD(q))
				continue;

			q = pj.get_GD_result();

			if (withObs && !pj.check_angle_limits(q))
				continue;

			break;
		}

		cout << "Sampled " << i+1 << " out of " << N << endl;

		pj.log_q(q);

		for (int j = 0; j < q.size(); j++)
			File << q[j] << " ";
		File << endl;
	}

	File.close();

	return 0;
}
