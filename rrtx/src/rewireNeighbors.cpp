#include "algorithms.hpp"
#include "functions.hpp"
#include <algorithm>

void rewireNeighbors(int someNodeIndex, Matrix &graph, Matrix &Q, N &neighbors, Matrix newDist, Row gValue, Row &lmc, double r, double epsilon, nav_msgs::OccupancyGrid mapsub)
{

	cullNeighbors(graph, someNodeIndex, neighbors, r, newDist);

	int Parent = graph[someNodeIndex][3];

	vector<int> wholeMinusNeighbors;

	wholeMinusNeighbors = neighbors.original_minus.at(someNodeIndex);
	for (int i = 0; i < neighbors.running_minus.at(someNodeIndex).size(); i++)
	{
		wholeMinusNeighbors.push_back(neighbors.running_minus.at(someNodeIndex).at(i));
	}

	int whichOne = findInVec(wholeMinusNeighbors, Parent);
	if (whichOne != -1)
	{
		vector<int>::iterator deletingParent;
		deletingParent = wholeMinusNeighbors.begin() + whichOne;
		wholeMinusNeighbors.erase(deletingParent);
	}

	for (int j = 0; j < wholeMinusNeighbors.size(); j++)
	{

		int u = wholeMinusNeighbors[j];

		if (lmc[u] > newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[someNodeIndex])
		{
			//int check = costmapObstacleCheck(Row{graph[u][0], graph[u][1]}, Row{graph[someNodeIndex][0], graph[someNodeIndex][1]}, mapsub);
			//if (check == 0)
			//	ROS_WARN("shiiiiiiiiiiiiiiiiiiiiiiiit");
			lmc[u] = newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[someNodeIndex];

			graph[u][3] = someNodeIndex;

			Row check_node = {graph[u][0], graph[u][1]};
			Row current_node = {graph[someNodeIndex][0], graph[someNodeIndex][1]};
			if (gValue[u] - lmc[u] > epsilon)
				verrifyQueue(Q, gValue[u], lmc[u], u);
		}
	}
}
