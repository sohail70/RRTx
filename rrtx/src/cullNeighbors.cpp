#include "algorithms.hpp"
#include <algorithm>

using namespace std;
void cullNeighbors(Matrix graph, double someNodeIndex, N &neighbors, double r, Matrix newDist)
{
	int size = neighbors.running_plus[someNodeIndex].size();
	int i = 0;

	while (i < size)
	{

		double neigh = neighbors.running_plus[someNodeIndex].at(i);

		if ((newDist[max(neigh, someNodeIndex)][min(neigh, someNodeIndex)] > r) && (graph[someNodeIndex][3] != neigh))
		{
			vector<int>::iterator deletingNeigh;
			deletingNeigh = neighbors.running_plus.at(someNodeIndex).begin() + i;
			neighbors.running_plus.at(someNodeIndex).erase(deletingNeigh);

			int whichOne = findInVec(neighbors.running_minus.at(neigh), someNodeIndex);
			if (whichOne != -1)
			{
				deletingNeigh = neighbors.running_minus.at(neigh).begin() + whichOne;
				neighbors.running_minus.at(neigh).erase(deletingNeigh);
			}

			size = neighbors.running_plus[someNodeIndex].size();
		}
		else
		{
			i++;
		}
	}
}
