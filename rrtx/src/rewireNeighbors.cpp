#include "algorithms.hpp"
#include "functions.hpp"
#include <algorithm>

//CHECK THE ALGORITHM PIECE BY PIECE

void rewireNeighbors(int someNodeIndex, Matrix &graph, Matrix &Q, N &neighbors, Matrix newDist, Row gValue, Row &lmc, double r, double epsilon, nav_msgs::OccupancyGrid mapsub)
{
	//MAYBE TODO: there is checker in my MATLAB version which I used in this function---but do i really need it? maybe I should just ignore the
	//sampling node which are in newly found obstacles-- or maybe not!
	ROS_WARN("alef");
	//cullNeighbors(graph, someNodeIndex, neighbors, r, newDist);

	//parent of someNodeIndex
	int Parent = graph[someNodeIndex][3]; //index of that parent
	//exclude parent from neighbors of the someNodeIndex
	vector<int> wholeMinusNeighbors; //whole minus neighbors of someIndexNode without the parent node of someIndexNode

	wholeMinusNeighbors = neighbors.original_minus.at(someNodeIndex);
	for (int i = 0; i < neighbors.running_minus.at(someNodeIndex).size(); i++) //adding the running minus to the original ones to have a MINUS PACKAGE
	{
		wholeMinusNeighbors.push_back(neighbors.running_minus.at(someNodeIndex).at(i));
	}

	//deleting the parent index from wholeMinusNeighbors indexes--->we could have used If statement in the next loop and didn't bother with the findInvec nonsense :(

	int whichOne = findInVec(wholeMinusNeighbors, Parent);
	if (whichOne != -1)
	{
		vector<int>::iterator deletingParent;
		deletingParent = wholeMinusNeighbors.begin() + whichOne;
		wholeMinusNeighbors.erase(deletingParent);
	}
	//ROS_WARN("beee");
	for (int j = 0; j < wholeMinusNeighbors.size(); j++) //wholeMinusNeighbors is a vector of indexes of neighbor nodes of someIndexNode except the parent node of a someIndexNode
	{

		int u = wholeMinusNeighbors[j]; //i'th neighbor node of the someNodeIndex
		//ROS_WARN("is it gonna change %f %f", lmc[u],newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[someNodeIndex]);
		if (lmc[u] > newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[someNodeIndex])
		{
			//update the lmc of neighbor node--> because someNodeIndex gives u a better path
			//	ROS_WARN("yes,changed the parent of %i from %f to %i and before lmc:%f newlmc:%f",u,graph[u][3],someNodeIndex,lmc[u],newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[someNodeIndex]);
			int check = costmapObstacleCheck(Row{graph[u][0], graph[u][1]}, Row{graph[someNodeIndex][0], graph[someNodeIndex][1]}, mapsub); //if you want to obstacle check with the costmap.
			//is the check_node(previus nodes) in obs?if it is, newDist->inf
			if (check == 0)
				ROS_WARN("shiiiiiiiiiiiiiiiiiiiiiiiit");
			lmc[u] = newDist[max(u, someNodeIndex)][min(u, someNodeIndex)] + lmc[someNodeIndex];

			graph[u][3] = someNodeIndex;

			//debug
			Row check_node = {graph[u][0], graph[u][1]};
			Row current_node = {graph[someNodeIndex][0], graph[someNodeIndex][1]};
			double val = euc_dist(check_node, current_node);
			if (val > 1.000001)
				//ROS_WARN("valllllllllllllllllllllll %f %i %i %f %f %f %f", val,u,someNodeIndex,graph[u][0],graph[u][1],graph[someNodeIndex][0],graph[someNodeIndex][1]);
				ROS_WARN("whyyyyyyyyyyyyyyyyyy %f", newDist[max(u, someNodeIndex)][min(u, someNodeIndex)]);
			if (gValue[u] - lmc[u] > epsilon)
				verrifyQueue(Q, gValue[u], lmc[u], u);
		}
	}
	//ROS_WARN("peeee");
}
