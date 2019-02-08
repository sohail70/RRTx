#include "algorithms.hpp"
using namespace std;

void updateObstacles(nav_msgs::OccupancyGrid mapsub, Matrix &graph, Matrix &Q, N &neighbors, Matrix &newDist, Row gValue, Row &lmc, vector<int> orphansIndex, double r, double epsilon, double robotNode, vector<double> closeNodesIndex)
{
	/*
	if (obstacleRemoved == true)
	{
		//for(int i=0;i<size of removed obstacles;i++) //baste be in dare ke node marbot be detection chanta chiz publish kune!
		{
			removeObstacle(mapsub, graph, Q, neighbors, newDist, gValue, lmc, r, orphansIndex);
		}
		reduceInconsistency(Q, graph, neighbors, newDist, gValue, lmc, robotNode, r, epsilon, orphansIndex,mapsub);
	}
	*/
	addNewObstacle(mapsub, graph, Q, neighbors, newDist, gValue, lmc, r, orphansIndex, robotNode, closeNodesIndex);
	propagateDescendants(graph, Q, neighbors, orphansIndex, gValue, lmc);
	if (robotNode != numeric_limits<double>::infinity())
		verrifyQueue(Q, gValue[robotNode], lmc[robotNode], robotNode);

	reduceInconsistency(Q, graph, neighbors, newDist, gValue, lmc, robotNode, r, epsilon, orphansIndex, mapsub);

}
