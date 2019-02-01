#include "rrtx_algorithms.h"
#include <map>
#include <algorithm>
using namespace std;

void addNewObstacle(obstacle& obsSet, Matrix& graph, Matrix& Q, N& neighbors, Matrix& newDist, Row gValue, Row& lmc, double r, vector<int>& orphansIndex, double robotNode, double safetyFactor)
{
	//add obstacle to obstacle set
	//obsSet.circles.push_back OR obsSet.lineSegs.push_back
	


	//create the E0--> messed up edges beacuse of the current obstacle
	
	//fek kunam loop haye paeen to method updateObstacles bashe behtare chun to addObs ham mishe az natayeje in loop estefade kard!
	vector<double> closeNodesIndex;
	//TODO:-->MY idea: faghat dakhele sensor radius obstacle set haroo negah kunim!...har moghe az senRad kharej shodi  obs ha remove she!

	for (int i = 0; i < obsSet.circles.size(); i++) //TODO: how to implement lineSegs!? what is a distance criteria?!
	{
		for (int j = 0; j < graph.size(); j++)
		{//TODO: I don't have to calculate the dist to all of the obstacles! just the added ones is enough!
			double val = euc_dist(graph[j], obsSet.circles[i]); //distance between all nodes to the center of all obstacles!
			if (val < safetyFactor*obsSet.circles[i][2]) //if val is lower that the radius of that obstacle-->so the node is in trouble!
				closeNodesIndex.push_back(j);
		}
	}


	//TODO---> try to merge the below loop to the upper one-->it's easy--->but don't do it now!--->after debugging!
	map<int, vector<int>> nodeToNeighsMap;//may be Unorder map for efficiency
										  //now we have V_close-->check all the neighbors of the each V_close to find the edge pair
	for (int i = 0; i < closeNodesIndex.size(); i++)
	{
		double currentOrphanNodeIndex = closeNodesIndex[i];

		vector<int> effectedNeighbors = neighbors.original_minus[currentOrphanNodeIndex];//for original minus(or plus-->doesn't matter for this implementation)
		vector<int> effectedNeighbors_running = neighbors.running_minus[currentOrphanNodeIndex];//for running minus of currentOrpahnNode
		effectedNeighbors.insert(effectedNeighbors.end(), effectedNeighbors_running.begin(), effectedNeighbors_running.end());//merging the two above

		nodeToNeighsMap[currentOrphanNodeIndex] = effectedNeighbors;//map from the messed up node to it's neighbors-->consider it the edges!--> E0 for current obstacle!
	}



	//for loop
	map<int, vector<int>>::iterator itr;
	itr = nodeToNeighsMap.begin();

	for (auto i : nodeToNeighsMap) //i for the node v
	{
		for (int j = 0; j < i.second.size(); j++)//j for the u nodes!(neighbors of v)
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