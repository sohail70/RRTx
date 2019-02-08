#include "algorithms.hpp"
#include "functions.hpp"

Row findParent(Row current_node, Matrix nearest_nodes, Row dist_to_near_nodes, Row &lmc, vector<int> andix, int current_node_index, nav_msgs::OccupancyGrid mapsub)
{
	tuple<double, int> min_dist;
	Row fasele;
	for (int j = 0; j < nearest_nodes.size(); j++)
	{
		fasele.push_back(dist_to_near_nodes[j] + lmc[andix[j]]);
	}

	if (nearest_nodes.size() != 0)
	{

		min_dist = LargestOrSmallestElement(fasele, fasele.size(), "smallest");
		double calc = get<0>(min_dist);
		int k = get<1>(min_dist);

		int check = costmapObstacleCheck(current_node, Row{nearest_nodes[k][0], nearest_nodes[k][1]}, mapsub);

		if (calc == numeric_limits<double>::infinity())
		{
			lmc.push_back(-1);
			return {current_node[0], current_node[1], (double)current_node_index, (double)-1};
		}

		lmc.push_back(calc);
		Row parent_of_v = {current_node[0], current_node[1], (double)current_node_index, (double)andix[k]};
		return parent_of_v;
	}
	else
	{

		lmc.push_back(-1);
		return {current_node[0], current_node[1], (double)current_node_index, (double)-1};
	}
}
