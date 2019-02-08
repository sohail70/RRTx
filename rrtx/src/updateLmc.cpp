#include "algorithms.hpp"
#include <algorithm>

void updateLmc(int someNodeIndex, Matrix& graph ,Matrix newDist,N neighbors ,Row& lmc , double r , vector<int> orphansIndex)
{
	
	cullNeighbors(graph, someNodeIndex, neighbors, r, newDist);

	vector<int> wholePlusNeighbors;
	wholePlusNeighbors = neighbors.original_plus.at(someNodeIndex);
	
	for (int i = 0; i < neighbors.running_plus.at(someNodeIndex).size(); i++) 
	{
		wholePlusNeighbors.push_back(neighbors.running_plus.at(someNodeIndex).at(i));
	}
	
	
	vector<int>::iterator deletingOrphans;
	for (int i = 0; i < orphansIndex.size(); i++)
	{
		int whichOne = findInVec(wholePlusNeighbors, orphansIndex[i]);
		if (whichOne != -1)
		{
			deletingOrphans = wholePlusNeighbors.begin() + whichOne;
			wholePlusNeighbors.erase(deletingOrphans);
		}
	}

	for (int i = 0; i < wholePlusNeighbors.size(); i++)
	{
		int u = wholePlusNeighbors[i];
		if (graph[u][3] != someNodeIndex)
			if (lmc[someNodeIndex] > newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[u])
			{
				graph[someNodeIndex][3] = u;
				lmc[someNodeIndex] = newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[u];
			}
	}
}
