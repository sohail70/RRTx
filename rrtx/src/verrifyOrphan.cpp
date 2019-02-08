#include "algorithms.hpp"

using namespace std;


void verrifyOrphan(double someNodeIndex, Matrix& Q, vector<int>& orphansIndex)
{
	for(int i=0;i<Q.size();i++)
		if (Q[i][2] == someNodeIndex)
		{
			Matrix::iterator deleteRow;
			deleteRow = Q.begin()+i;
			Q.erase(deleteRow);
		}
	orphansIndex.push_back(someNodeIndex);
}
