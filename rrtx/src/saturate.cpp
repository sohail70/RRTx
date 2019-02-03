#include "algorithms.hpp"

void saturate(vector<double> &unsat_node, vector<double> nearest_node, double step_size)
{
	if (euc_dist(unsat_node, nearest_node) < step_size)
	{
		return;
	}
	else
	{
		double theta = atan2(unsat_node[1] - nearest_node[1], unsat_node[0] - nearest_node[0]);
		unsat_node[0] = nearest_node[0] + step_size * cos(theta);
		unsat_node[1] = nearest_node[1] + step_size * sin(theta);
	}
}
