#include "algorithms.hpp"
#include "functions.hpp"
#include <iomanip>
Row extend(Matrix &graph, double r, Row &lmc, N &neighbors, nav_msgs::OccupancyGrid mapsub)
{
	Row new_distance;
	Matrix near_nodes;
	Row dist_to_near_nodes;

	vector<int> andix;
	int i = graph.size() - 1;

	neighbors.original_plus.resize(i + 1);
	neighbors.original_minus.resize(i + 1);
	neighbors.running_plus.resize(i + 1);
	neighbors.running_minus.resize(i + 1);

	int counter = 0;
	Row current_node = {graph[i][0], graph[i][1]};

	bool CurrentNodeIsInObs = false;

	if (gridValue(mapsub, current_node) > 0)
		CurrentNodeIsInObs = true;

	bool checkNodeIsInObs = false;

	if (CurrentNodeIsInObs == false)
	{
		for (int j = 0; j < i; j++)
		{
			Row check_node = {graph[j][0], graph[j][1]};

			float val = euc_dist(check_node, current_node);

			checkNodeIsInObs = false;
			if (val <= r)
			{
				int check = costmapObstacleCheck(current_node, check_node, mapsub);

				if (check == 0)
					checkNodeIsInObs = true;
			}

			if (checkNodeIsInObs == true || val > r)
				new_distance.push_back(numeric_limits<double>::infinity());
			else
				new_distance.push_back(val);

			if (val <= r)
			{
				andix.push_back(j);
				dist_to_near_nodes.push_back(new_distance[j]);
				near_nodes.push_back(graph[andix[counter]]);

				neighbors.original_plus[i].push_back(andix[counter]);
				neighbors.original_minus[i].push_back(andix[counter]);
				neighbors.running_plus[andix[counter]].push_back(i);
				neighbors.running_minus[andix[counter]].push_back(i);

				counter += 1;
			}
		}
	}
	graph[i] = findParent(current_node, near_nodes, dist_to_near_nodes, lmc, andix, i, mapsub);

	if (graph[i][3] == (-1) || CurrentNodeIsInObs == true)
	{

		Matrix::iterator useless_node;
		useless_node = graph.end() - 1;
		graph.erase(useless_node);
		lmc.erase(lmc.end() - 1);
		return new_distance;
	}
	return new_distance;
}
