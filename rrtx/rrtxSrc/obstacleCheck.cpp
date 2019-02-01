#include "rrtx_algorithms.h"
#include "magic_algorithms.h"
//check if somenodePosition is in the obstacle region -->consider safetyFactor!
bool obstacleCheck(Row position, obstacle obs, double safetyFactor)
{
	bool vIsInObs=false;
	
	//TODO: what about line segments!
	for (int i = 0; i < obs.circles.size(); i++)//loop over whole existing obstacles
	{
		Row currentObs = { obs.circles[i][0],obs.circles[i][1] };
		double val = euc_dist(position, currentObs);
		if (val < safetyFactor*obs.circles[i][2])
		{
			vIsInObs = true;
			break;//don't waste time checking other obstacles--> v is in obstacle region, period!
		}
	}
	return vIsInObs;
}