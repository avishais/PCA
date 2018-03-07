/**
 * @file pca_example.cpp
 * @brief This is an example program demonstrating the usage of the pca class
 */
#include "pca.h"
#include "grassmann_pca.hpp"
#include <iostream>
#include <random>

using namespace std;

int main()
{

	srand( time(NULL) );

	const int num_variables = 2;
	const int num_records = 11;

	stats::pca pca(num_variables, num_records);
	pca.set_do_bootstrap(false, 100);

	double T = 0;
	int N = 1;
	for (int j = 0; j < N; j++)
	{

		clock_t st = clock();
		pca.set_num_records(num_records);
		cout << "Clear time: " << double(clock() - st) / CLOCKS_PER_SEC << endl;

		cout << "Adding random data records ..." << endl;
		ifstream mf;
		mf.open("samples.txt");
		
		for (int i = 0; i < num_records; ++i)
		{
			int j = 0;
			vector<double> record(num_variables);
			for (auto &value : record)
			{
				// value = ((double)rand()/RAND_MAX) * 20 - 10;
				mf >> value;
				//cout << value << " ";
				j++;
			}
			//cout << endl;			

			pca.add_record(record);
		}
		mf.close();

		cout << "Solving ..." << endl;
		st = clock();
		pca.solve();
		T += double(clock() - st) / CLOCKS_PER_SEC;

		
		//pca.clear_records();
		
	}
	
	cout << T / N << endl;

	//for (int j = 0; j < num_variables; j++) 
	//	cout << pca.get_eigenvalue(j) << endl;

	/*cout << "Energy = " << pca.get_energy() << " (" << stats::utils::get_sigma(pca.get_energy_boot()) << ")" << endl;

	const auto eigenvalues = pca.get_eigenvalues();
	cout << "First three eigenvalues = " << eigenvalues[0] << ", "
		 << eigenvalues[1] << ", "
		 << eigenvalues[2] << endl;

	cout << "Orthogonal Check = " << pca.check_eigenvectors_orthogonal() << endl;
	cout << "Projection Check = " << pca.check_projection_accurate() << endl;*/

	pca.save("pca_results");

	// -------------------------

	grassmann_averages_pca::grassmann_pca<vector<double>> gpca;

	// gpca.batch_process<data_t, 


	

	return 0;
}
