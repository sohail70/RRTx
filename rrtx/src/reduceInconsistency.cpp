#include "algorithms.hpp"
#include "functions.hpp"
#include <algorithm>

int reduceInconsistency(Matrix& Q,Matrix& graph,N& neighbors, Matrix newDist , Row& gValue , Row& lmc , double robotNode , double r, double epsilon, vector<int> orphansIndex,nav_msgs::OccupancyGrid mapsub)
{
	bool isVbotInQueue; 
	Row checker;
	checker = find(Q , (int)robotNode);
	
	if (checker[0]!=-1)
		isVbotInQueue = true;
	else
		isVbotInQueue = false;



	bool isRobNodeInconsistent; 
	
	if (robotNode == numeric_limits<double>::infinity())
		isRobNodeInconsistent = true;
	else
	{
		if (lmc[robotNode] != gValue[robotNode])
			isRobNodeInconsistent = true;
		else
			isRobNodeInconsistent = false;
	}


	bool robGvalueCheck;
	if (robotNode == numeric_limits<double>::infinity())
		robGvalueCheck = true;
	else
	{
		if (gValue[robotNode] == numeric_limits<double>::infinity()) 
			robGvalueCheck = true;
		else 
			robGvalueCheck = false;
	}

	bool shouldReduce=false;
	Row keyOne;
	Row keyTwo;
	if ( ! Q.empty() && robotNode != numeric_limits<double>::infinity())
	{
		keyOne = { min(Q[0][0], Q[0][1]) ,Q[0][1] }; 
		keyTwo = { min(gValue[robotNode], lmc[robotNode]) ,gValue[robotNode] };
		if (keyOne[0] == keyTwo[0])
		{
			if (keyOne[1] < keyTwo[1]) 
				shouldReduce = true;
		}
		else if (keyOne[0] < keyTwo[0])
		{
			shouldReduce = true;
		}
	}
	int counter = 0;

	while ( Q.size() > 0 &&   (shouldReduce ||isRobNodeInconsistent || robGvalueCheck || isVbotInQueue) )
	{
		if (gValue[Q[0][2]] - lmc[Q[0][2]] > epsilon) 
		{
			updateLmc(Q[0][2],graph, newDist,neighbors,lmc, r, orphansIndex);
			rewireNeighbors((int)Q[0][2], graph, Q, neighbors, newDist, gValue, lmc, r, epsilon,mapsub);
		}
		
		gValue[Q[0][2]] = lmc[Q[0][2]];
		
		
		Matrix::iterator deleteRow;
		deleteRow = Q.begin();
		Q.erase(deleteRow);

		if (!Q.empty()) 
		{
			
			keyOne = { min(Q[0][0], Q[0][1]) ,Q[0][1] };
			keyTwo = { min(gValue[robotNode], lmc[robotNode]) ,gValue[robotNode] };
			if (keyOne[0] == keyTwo[0])
			{
				if (keyOne[1] > keyTwo[1])
					shouldReduce = false;
			}
			else if (keyOne[0] > keyTwo[0])
			{
				shouldReduce = false;
			}
			
			if (lmc[robotNode] == gValue[robotNode])
				isRobNodeInconsistent = false;

			
			if (gValue[robotNode] != numeric_limits<double>::infinity())
				robGvalueCheck = false;
			
			checker = find(Q, (int)robotNode);
			if (checker[0] == -1)
				isVbotInQueue = false;
		}
		counter += 1;
	}

	return counter;
}
