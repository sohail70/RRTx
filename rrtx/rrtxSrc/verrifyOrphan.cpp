#include "rrtx_algorithms.h"

using namespace std;


void verrifyOrphan(double someNodeIndex, Matrix& Q, vector<int>& orphansIndex)
{
	//if the current node is in pririty Q, delete it from the queue, because this node is in a messed up situation
	for(int i=0;i<Q.size();i++)
		if (Q[i][2] == someNodeIndex)
		{
			//delete the i'th row of the priority Queue
			Matrix::iterator deleteRow;
			deleteRow = Q.begin()+i;
			Q.erase(deleteRow);
		}
	//add to orphanage
	orphansIndex.push_back(someNodeIndex);
}