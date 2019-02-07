#include "algorithms.hpp"
#include <map>
#include <algorithm>
using namespace std;

void addNewObstacle(nav_msgs::OccupancyGrid mapsub, Matrix &graph, Matrix &Q, N &neighbors, Matrix &newDist, Row gValue, Row &lmc, double r, vector<int> &orphansIndex, double robotNode,vector<double> closeNodesIndex)
{
	
	map<int, vector<int>> nodeToNeighsMap; //may be Unorder map for efficiency
										   //now we have V_close-->check all the neighbors of the each V_close to find the edge pair
	for (int i = 0; i < closeNodesIndex.size(); i++)
	{
		double currentOrphanNodeIndex = closeNodesIndex[i];

		vector<int> effectedNeighbors = neighbors.original_minus[currentOrphanNodeIndex];									   //for original minus(or plus-->doesn't matter for this implementation)
		vector<int> effectedNeighbors_running = neighbors.running_minus[currentOrphanNodeIndex];							   //for running minus of currentOrpahnNode
		effectedNeighbors.insert(effectedNeighbors.end(), effectedNeighbors_running.begin(), effectedNeighbors_running.end()); //merging the two above
		//for(int k=0;k<effectedNeighbors.size();k++)
			//ROS_WARN("efffffffffeeeeeeeected neighbor %i",effectedNeighbors[k]);
		nodeToNeighsMap[currentOrphanNodeIndex] = effectedNeighbors; //map from the messed up node to it's neighbors-->consider it the edges!--> E0 for current obstacle!
	}

	//for loop
	map<int, vector<int>>::iterator itr;
	itr = nodeToNeighsMap.begin();

	for (auto i : nodeToNeighsMap) //i for the node v
	{
		for (int j = 0; j < i.second.size(); j++) //j for the u nodes!(neighbors of v)
		{
			//inf the distance of the messed up edges
			newDist[max(i.first, i.second[j])][min(i.first, i.second[j])] = numeric_limits<double>::infinity();
			if (graph[i.first][3] == i.second[j])
				verrifyOrphan(i.first, Q, orphansIndex);
			//if (robotNode ==i.first || robotNode ==i.second[j])
			//In this critical case robot should stop, because the path the robot is taking is compromised, BUT STOP till WHEN??!!!!!
		}
	}
}
