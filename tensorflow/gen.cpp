#include "../utils/pca.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <random>
#include <math.h>

// For nn-search
#include "./simple/nanoflann.hpp"
#include "./simple/KDTreeVectorOfVectorsAdaptor.h"

using namespace std;
using namespace nanoflann;

typedef vector<double> Vector;
typedef std::vector< int > VectorInt;
typedef vector<Vector> Matrix;

typedef KDTreeVectorOfVectorsAdaptor< Matrix, double > my_kd_tree_t;

const int num_variables = 12;
const int num_records = 1e6;

struct kNeighborSoln {
	VectorInt neighbors;
	Vector dist;
};

template <typename num_t>
void printVector(vector<num_t> V) {
    for (int i = 0; i < V.size(); i++)
        cout << V[i] << " ";
    cout << endl;
}


Matrix load_samples() {
    ifstream mf;
	mf.open("/home/avishai/Documents/workspace/ml_planner/data/abb_samples_noJL.db");

    cout << "Loading data..." << endl;
    Matrix M;
    for (int i = 0; i < num_records; ++i)
    {
        int j = 0;
        vector<double> record(num_variables);
        for (auto &value : record)
        {
            mf >> value;
            j++;
        }

        M.push_back(record);
    }

    mf.close();
    cout << "Data loaded." << endl;

    return M;
}

VectorInt kNeighbors(my_kd_tree_t& mat_index, Vector query, kNeighborSoln& soln, double search_radius, bool remove_1st_neighbor = false){
	// find nearest neighbors with in radius 'search_radius'

    std::vector<std::pair<size_t,double> >   ret_matches;
    nanoflann::SearchParams params;

	Vector query_pnt = query;
	//mat_index.index->findNeighbors(resultSet, &query_pnt[0], SearchParams(10));
	const size_t nMatches = mat_index.index->radiusSearch(&query_pnt[0], search_radius, ret_matches, params);

	VectorInt rtn_values(nMatches);
    for (size_t i = 0; i < nMatches; i++)
        rtn_values[i] = ret_matches[i].first;

	if (remove_1st_neighbor) {
		rtn_values.erase(rtn_values.begin()); // Remove first node that is itself.
	}

	return rtn_values;
}

void run(Matrix M) {

    ofstream mf;
	mf.open("samplesTangent.db", std::ofstream::app);

    my_kd_tree_t KDtree(num_variables, M, 10);
    KDtree.index->buildIndex();
    kNeighborSoln neighborSearch;

    stats::pca pca(num_variables);
	pca.set_do_bootstrap(false, 100);

    for (int i = 0; i < 1+0*num_records; i++) {
        cout << "Completed " << (double)i/num_records*100 << "%...\n";

        // for (int j = 0; j < num_variables; j++)
        //     mf << M[i][j] << " ";
        // mf << endl;

        VectorInt I = kNeighbors(KDtree, M[i], neighborSearch, 4.0, false);
        cout << I.size() << endl;
        // pca.set_num_records(I.size());
        // for (int j = 0; j < I.size(); j++) 
        //     pca.add_record(M[I[j]]);

        // pca.solve();

        // for (int j = 0; j < num_variables; j++) {
        //     Vector eigv = pca.get_eigenvector(j);
        //     mf << eigv[0] << " " << eigv[1] << " " << eigv[2] << " ";
        // }

        // Vector eigc = pca.get_eigenvalues();
        // mf << (eigc[2] / sqrt( eigc[0]*eigc[0] + eigc[1]*eigc[1] + eigc[2]*eigc[2] )) << endl;
    }

    mf.close();
}


int main() {
    Matrix M = load_samples();
    run(M);
    
    return 0;
}