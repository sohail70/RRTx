#include "algorithms.hpp"
#include <algorithm>

using namespace std;
//MUST BE CHECKED PIECE BY PIECE!!!!!!
void cullNeighbors(Matrix graph , double someNodeIndex , N& neighbors , double r , Matrix newDist)
{
	//ROS_WARN("qqq");
	int size = neighbors.running_plus[someNodeIndex].size();
	int i = 0;

	while(i<size)
	{
		
		double neigh = neighbors.running_plus[someNodeIndex].at(i); //node index of the i'th neighbor of someNodeIndex
		//newDist has a unique logic inside it so I need to be carefull and check before
		//indexing inside it-->so use MIN and MAX function

		if ((newDist[max(neigh, someNodeIndex)][min(neigh, someNodeIndex)] > r) && (graph[someNodeIndex][3] != neigh))
		{
			vector<int>::iterator deletingNeigh;
			deletingNeigh = neighbors.running_plus.at(someNodeIndex).begin()+i;//I guess begin must be there to make it an iterator
			neighbors.running_plus.at(someNodeIndex).erase(deletingNeigh); //deleting u from running neighbors of v---> because r is decreased now
			
//ROS_WARN("qqqqqq");
			int whichOne = findInVec(neighbors.running_minus.at(neigh), someNodeIndex);
			if (whichOne != -1)
			{
				deletingNeigh = neighbors.running_minus.at(neigh).begin() + whichOne;
				neighbors.running_minus.at(neigh).erase(deletingNeigh); // deleting v from u's running neghbors--> because r is decreased now
			}

			size = neighbors.running_plus[someNodeIndex].size(); //if it's erased; size must be decreased so update it!
		}
		else
		{
			i++;
		}
		
	}
	//ROS_WARN("qqqqqqqqqqqqqqqqq");
}
