#include "kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

bool withObs = true;

int main() {

	int Seed = time(NULL);
	srand(Seed);
	cout << "Seed in testing: " << Seed << endl;

	std::ofstream File;
	File.open("abb_samples_JL.txt", ios::app);

	// KDL
	kdl pj;

	int N = 2e6;
	State qb4(12), qaf(12);
	for (int i = 0; i < N; i++) {

		
		while (1) {
			qb4 = pj.rand_q();

			if (!pj.GD(qb4))
				continue;

			qaf = pj.get_GD_result();

			if (withObs && !pj.check_angle_limits(qaf))
				continue;

			break;
		}

		cout << "Completed " << double(i+1)/N*100 << "%%." << endl;

		pj.log_q(qaf);

		for (int j = 0; j < qb4.size(); j++)
			File << qb4[j] << " ";
		for (int j = 0; j < qaf.size(); j++)
			File << qaf[j] << " ";
		File << endl;
	}

	File.close();

	return 0;
}
