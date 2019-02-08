#include "algorithms.hpp"
#include <map>
#include <algorithm>
using namespace std;

void addNewObstacle(nav_msgs::OccupancyGrid mapsub, Matrix &graph, Matrix &Q, N &neighbors, Matrix &newDist, Row gValue, Row &lmc, double r, vector<int> &orphansIndex, double robotNode, vector<double> closeNodesIndex)
{

	map<int, vector<int>> nodeToNeighsMap;

	for (int i = 0; i < closeNodesIndex.size(); i++)
	{
		double currentOrphanNodeIndex = closeNodesIndex[i];

		vector<int> effectedNeighbors = neighbors.original_minus[currentOrphanNodeIndex];
		vector<int> effectedNeighbors_running = neighbors.running_minus[currentOrphanNodeIndex];
		effectedNeighbors.insert(effectedNeighbors.end(), effectedNeighbors_running.begin(), effectedNeighbors_running.end());
		nodeToNeighsMap[currentOrphanNodeIndex] = effectedNeighbors;
	}

	map<int, vector<int>>::iterator itr;
	itr = nodeToNeighsMap.begin();

	for (auto i : nodeToNeighsMap)
	{
		for (int j = 0; j < i.second.size(); j++)
		{
			newDist[max(i.first, i.second[j])][min(i.first, i.second[j])] = numeric_limits<double>::infinity();
			if (graph[i.first][3] == i.second[j])
				verrifyOrphan(i.first, Q, orphansIndex);
		}
	}
}
