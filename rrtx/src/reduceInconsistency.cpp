#include "algorithms.hpp"
#include "functions.hpp"
#include <algorithm>

//Check it piece by piece!
int reduceInconsistency(Matrix& Q,Matrix& graph,N& neighbors, Matrix newDist , Row& gValue , Row& lmc , double robotNode , double r, double epsilon, vector<int> orphansIndex,nav_msgs::OccupancyGrid mapsub)
{
	bool isVbotInQueue; //robots node is important! so if it is in the priority queue! make it
	Row checker;
	checker = find(Q , (int)robotNode);// you can return an empty vector you know....check kun kojaha az find estefade kardi ke badan algorithm ro taghir bedi! (-1,-1) jaleb nist!
	
	if (checker[0]!=-1)
		isVbotInQueue = true;
	else
		isVbotInQueue = false;



	bool isRobNodeInconsistent; //it means there is inconsistency for robot node and we need to update the gValue of the robots node(may be there is a better way or maybe obstalce is ahead that makes our robot node inconsistent)
	
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
	if (robotNode == numeric_limits<double>::infinity())//it means the tree is not reached the current value of the robot so !
		robGvalueCheck = true;
	else //when the tree reached the robots current position!
	{
		if (gValue[robotNode] == numeric_limits<double>::infinity()) //we must reduce consistency if the gValue of the robots node is inf or else the robot is getting confused where to go(I guess)
			robGvalueCheck = true;
		else //when we have a gValue for robots current node, we are safe and doesnt need to reduce consistency of Queue!(don't waste time reducing it!)---> mage inke niaz beshe!(manzoor sharthaye dg hast)
			robGvalueCheck = false;
	}
//shouldReduce true?  key of LPA star
//U:TopKey < CalulateKey(sgoal)
//[min(g(s); rhs(s)) + h(s); min(g(s); rhs(s))]

	bool shouldReduce=false;
	Row keyOne;
	Row keyTwo;
	if ( ! Q.empty() && robotNode != numeric_limits<double>::infinity())
	{
		//the key in RRTx is [min(g(v),lmc(v)); g(v)]
		//key of the top node in priority queue
		keyOne = { min(Q[0][0], Q[0][1]) ,Q[0][1] }; //albate fek kunam verrify queue eeee ke to main gozashti inkaro mikune barat va hamoon element aval doroste!
		//key of the robots current node
		keyTwo = { min(gValue[robotNode], lmc[robotNode]) ,gValue[robotNode] };
		if (keyOne[0] == keyTwo[0])
		{
			if (keyOne[1] < keyTwo[1]) //it means the robots gValue(or lmc) is too high than the top priority queue--> meaning that the cascade did'nt reach the robs node--> so reduce inconsistency more and more
				shouldReduce = true;
		}
		else if (keyOne[0] < keyTwo[0])
		{
			shouldReduce = true;
		}
	}
	int counter = 0;
	//Now the algorithm----> may be before obstacle detection there shoudl'nt be any reduce inconsistency! what's the point? huh!-->make conditions false!
	while ( Q.size() > 0 &&   (shouldReduce ||isRobNodeInconsistent || robGvalueCheck || isVbotInQueue) )
	{
		//ROS_WARN("1");
		if (gValue[Q[0][2]] - lmc[Q[0][2]] > epsilon) //Q[0][2] is the node Index That we must make consistent---> Q[0][2] is the v in reduceInconsistency algorithm in the paper!
		{
			//ROS_WARN("2");
			updateLmc(Q[0][2],graph, newDist,neighbors,lmc, r, orphansIndex);
			//ROS_WARN("3");
			rewireNeighbors((int)Q[0][2], graph, Q, neighbors, newDist, gValue, lmc, r, epsilon,mapsub); //check for pass by reference values (use the paper!)
			//ROS_WARN("4");
		}
		
		gValue[Q[0][2]] = lmc[Q[0][2]];
		
		//delete the first row of the Queue
		Matrix::iterator deleteRow;
		deleteRow = Q.begin();
		Q.erase(deleteRow);

		if (!Q.empty()) //checking the conditions but if the Q is empty, it doesnt matter!
		{
			//ShouldReduce? just check for the false condition because it's already true when it's inside the loop---> be carefull, doubt the preceduer
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
			//isRobNodeInconsistent?
			if (lmc[robotNode] == gValue[robotNode])
				isRobNodeInconsistent = false;

			//is robGvalueCheck?
			if (gValue[robotNode] != numeric_limits<double>::infinity())
				robGvalueCheck = false;
			//isVbotInQueue?
			checker = find(Q, (int)robotNode);
			if (checker[0] == -1)
				isVbotInQueue = false;
		}
		counter += 1;
	}

	return counter;
}
