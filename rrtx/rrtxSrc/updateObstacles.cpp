#include "rrtx_algorithms.h"

using namespace std;

//given: obstacle set state!---> I need a struct for it -->categorizing the obstacles in two fields(circle/line seg)
//so in the end we have two sets--->first set is like this : set1={obs1_state,obs2_state,...}
//obs1_state=<x_obs1,y_obs1,estimated_radius> and so on
//obs2_state=<
void updateObstacles(obstacle& obsSet, Matrix& graph, Matrix& Q, N& neighbors, Matrix& newDist, Row gValue, Row& lmc, vector<int> orphansIndex, double r, double epsilon, double robotNode, double safetyFactor)
{
	
	bool obstacleAdded=false; //init
	bool obstacleRemoved=false;//init-->we need a loop before this function to check these two boolean!


	//for debuging 
	obstacleAdded = true;


	if (obstacleRemoved == true)
	{
		//for(int i=0;i<size of removed obstacles;i++) //baste be in dare ke node marbot be detection chanta chiz publish kune!
		{
			removeObstacle(obsSet, graph, Q, neighbors, newDist, gValue, lmc, r, orphansIndex, safetyFactor);
		}
		reduceInconsistency(Q, graph, neighbors, newDist, gValue, lmc, robotNode, r, epsilon, orphansIndex);
	}

	int counter;
	if (obstacleAdded == true)
	{

		//for (int i=0;i<size of added obstacle set;i++) //baste be publisher e node detection dare!
		{
			addNewObstacle( obsSet, graph, Q, neighbors, newDist, gValue, lmc, r, orphansIndex, robotNode, safetyFactor);
		}
		//if(!orphansIndex.empty())
		propagateDescendants( graph, Q, neighbors, orphansIndex, gValue, lmc);
		
		if(robotNode!=numeric_limits<double>::infinity())
			verrifyQueue(Q, gValue[robotNode], lmc[robotNode], robotNode);
		
		counter=reduceInconsistency(Q, graph, neighbors, newDist, gValue, lmc, robotNode, r, epsilon, orphansIndex);
		cout << "counter" << counter << endl;
	}


	



	//this part is for existing obstacles which have changed and I don't know how to implement it yet
	//my first idea is to discretize it to two steps and call remove&add subsequently!
	//maybe a better idea comes to mind in near future

}