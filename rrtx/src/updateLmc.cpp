#include "algorithms.hpp"
#include <algorithm>

//CHECK it PIECE BY PIECE!
void updateLmc(int someNodeIndex, Matrix& graph ,Matrix newDist,N neighbors ,Row& lmc , double r , vector<int> orphansIndex)
{
	cullNeighbors(graph, someNodeIndex, neighbors, r, newDist);
	

	vector<int> wholePlusNeighbors;//(original+running) plus neighbors of someNodeIndex
	wholePlusNeighbors = neighbors.original_plus.at(someNodeIndex);
	for (int i = 0; i < neighbors.running_plus.at(someNodeIndex).size(); i++) //adding the running plus to the original ones to have a PLUS PACKAGE
	{
		wholePlusNeighbors.push_back(neighbors.running_plus.at(someNodeIndex).at(i));
	}
	
	//deleting the orphanNodes Index from wholePlusNeighbors
	vector<int>::iterator deletingOrphans;
	for (int i = 0; i < orphansIndex.size(); i++)
	{
		int whichOne = findInVec(wholePlusNeighbors, orphansIndex[i]);
		if (whichOne != -1)
		{
			deletingOrphans = wholePlusNeighbors.begin() + whichOne;
			wholePlusNeighbors.erase(deletingOrphans); //albate in har bar ye element ro kam mikune
		}//vali jaye negarani nist chon dar har tekrar, whichOne dar vector fe@li donbale index migarde!
	}




	for (int i = 0; i < wholePlusNeighbors.size(); i++)
	{
		int u = wholePlusNeighbors[i];
		if (graph[u][3] != someNodeIndex) //age parent(u) !=v bod
			if (lmc[someNodeIndex] > newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[u])
			{
				graph[someNodeIndex][3] = u;
				lmc[someNodeIndex] = newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[u];
			}
		//should I calculate the lmc LIKE in MY MATLAB version!?? i guess you should! because it doesn't get updated any where else!
		//I think rewireNeighbor takes care of it and its reduntant doing it here but I'm not sure!

	}

}
