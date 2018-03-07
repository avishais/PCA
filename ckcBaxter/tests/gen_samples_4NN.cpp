#include "kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

bool withObs = false;

State rand_q() {
	State q(14);
	for (int i = 0; i < q.size(); i++)
		q[i] = (double)rand()/RAND_MAX * 2*PI_ - PI_;

	return q;
}

int main() {

	int Seed = time(NULL);
	srand(Seed);
	cout << "Seed in testing: " << Seed << endl;

	std::ofstream File;
	File.open("baxter_samples_noJL.txt", ios::app);

	// KDL
	kdl pj;

	int N = 2e6;
	State qb4(14), qaf(14);
	for (int i = 0; i < N; i++) {
		
		while (1) {
			qb4 = rand_q();

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
