#include "rrtx_algorithms.h"
#include <map>
#include <algorithm>
using namespace std;
//deghat kun ke E0 marbot be edge hayee hastan ke az current obs biroon omadan ya dakhelesh shodan!
void removeObstacle(obstacle& obsSet, Matrix& graph, Matrix& Q , N& neighbors , Matrix& newDist, Row gValue, Row& lmc, double r, vector<int> orphansIndex, double safetyFactor)
{
	//E0===>find the edges that were in collision with the removed obstacle(before the removal offcourse!)
	//in order to find the eges you need to find all the V_close nodes first and then check all the edges 
	//in neigbor sets of v_close then you can find all the nodes that are being effected
	


	//fek kunam loop haye paeen to method updateObstacles bashe behtare chun to addObs ham mishe az natayeje in loop estefade kard!
	vector<double> closeNodesIndex;
	//TODO:--->MY idea: faghat dakhele sensor radius obstacle set haroo negah kunim!...har moghe az senRad kharej shodi  obs ha remove she!
	for (int i = 0; i < obsSet.circles.size(); i++) //TODO: how to implement lineSegs!? what is a distance criteria?!
	{
		for (int j = 0; j < graph.size(); j++)
		{//TODO: I don't have to calculate the dist to all of the obstacles! just the removed ones is enough!
			double val = euc_dist(graph[j], obsSet.circles[i]); //distance between all nodes to the center of all obstacles!
			if (val <safetyFactor*obsSet.circles[i][2]) //if val is lower that the radius of that obstacle-->so the node is in trouble!
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
		vector<int> effectedNeighbors_running= neighbors.running_minus[currentOrphanNodeIndex];//for running minus of currentOrpahnNode
		effectedNeighbors.insert(effectedNeighbors.end(), effectedNeighbors_running.begin(), effectedNeighbors_running.end());//merging the two above
		
		nodeToNeighsMap[currentOrphanNodeIndex] = effectedNeighbors;//map from the messed up node to it's neighbors-->consider it the edges!--> E0 for current obstacle!
	}





	//remove this obstacle from the obstacle set--->(maybe)I should do it in updateObstacle method
	//obsSet.circles.erase  OR obsSet.lineSeg.erase 

	//remove edges from E0 which are still in some other not removed obstacles!----> I think this should be taken care of in updateObstacle method

	//now the for loop
	map<int, vector<int>>::iterator itr;
	itr = nodeToNeighsMap.begin();

	for (auto i : nodeToNeighsMap) //i for the node v
	{
		for (int j = 0; j < i.second.size(); j++)//j for the u nodes!(neighbors of v)
		{
			//how to index instruction:
			/*
			cout<<i.first<<"  "<<i.second[j];
			cout<<",";
			*/

			//recalculating the distance between the two freed up node (I mean, v and it's neighbor u)
			double val = euc_dist(graph[/*(double)*/i.first], graph[/*(double)*/i.second[j]]);
			newDist[max(i.first,i.second[j])][min(i.first,i.second[j])] = val;


		}
		updateLmc(/*(double)*/i.first, graph, newDist, neighbors, lmc, r, orphansIndex); //lmc was inf before , now the nodes been freed up, it's getting updated
		if (lmc[i.first] != gValue[i.first])
			verrifyQueue(Q, gValue[i.first], lmc[i.first], (double)i.first); //TODO: bebin bayad double bezari ya na! age bayad ke double to halghe yani vase val ham bezar!!!
	}

}
//I don't know why the graph configuration doesn't (exactly) go back to before when you remove an obstacle? maybe because r changes! -->NOT SURE!
//you need to think about the descendant too!!
