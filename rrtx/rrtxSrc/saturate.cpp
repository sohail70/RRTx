#include"rrtx_algorithms.h"


void  saturate(vector<double> &unsat_node,vector<double> nearest_node,double step_size)
{
	double theta = atan2(unsat_node[1] - nearest_node[1], unsat_node[0] - nearest_node[0]);
	unsat_node[0] = nearest_node[0] + step_size*cos(theta);
	unsat_node[1] = nearest_node[1] + step_size*sin(theta);
}

