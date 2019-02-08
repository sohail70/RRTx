#include "algorithms.hpp"
#include "functions.hpp"
#include <algorithm>
using namespace std;

void propagateDescendants(Matrix &graph, Matrix &Q, N &neighbors, vector<int> &orphansIndex, Row &gValue, Row &lmc)
{
	vector<int> temp(orphansIndex.begin(), orphansIndex.end());

	vector<int> vec;
	vector<int> currentChilds;
	vector<int> wholeChilds;
	int j = 1;

	while (!temp.empty())
	{
		vec.clear();
		vec = findRow(graph, temp[j - 1]);
		int whichOne = findInVec(vec, temp[j - 1]);
		vector<int>::iterator deletingNode;
		deletingNode = vec.begin() + whichOne;
		vec.erase(deletingNode);

		for (int i = 0; i < temp.size(); i++)
		{
			int whichOne = findInVec(vec, temp[i]);
			if (whichOne != -1)
			{
				vector<int>::iterator deletingNode;
				deletingNode = vec.begin() + whichOne;
				vec.erase(deletingNode);
			}
		}

		currentChilds.insert(currentChilds.end(), vec.begin(), vec.end());
		j++;

		if (j > temp.size())
		{
			temp = currentChilds;
			wholeChilds.insert(wholeChilds.end(), temp.begin(), temp.end());
			currentChilds.clear();
			currentChilds.resize(0);
			j = 1;
		}
	}
	orphansIndex.insert(orphansIndex.end(), wholeChilds.begin(), wholeChilds.end());

	vector<int> wholePlusNeighborsAndParent;

	for (int i = 0; i < orphansIndex.size(); i++)
	{
		wholePlusNeighborsAndParent = neighbors.original_plus.at(orphansIndex[i]);
		for (int k = 0; k < neighbors.running_plus.at(orphansIndex[i]).size(); k++)
		{
			wholePlusNeighborsAndParent.push_back(neighbors.running_plus.at(orphansIndex[i]).at(k));
		}

		ROS_WARN("graph %f %f %f %f", graph[orphansIndex[i]][0], graph[orphansIndex[i]][1], graph[orphansIndex[i]][2], graph[orphansIndex[i]][3]);
		if (graph[orphansIndex[i]][3] != numeric_limits<double>::infinity())
			wholePlusNeighborsAndParent.push_back(graph[orphansIndex[i]][3]);

		for (int j = 0; j < wholePlusNeighborsAndParent.size(); j++)
		{

			int u = wholePlusNeighborsAndParent[j];
			gValue[u] = numeric_limits<double>::infinity();
			verrifyQueue(Q, gValue[u], lmc[u], u);
		}
		gValue[orphansIndex[i]] = numeric_limits<double>::infinity();
		lmc[orphansIndex[i]] = numeric_limits<double>::infinity();
		graph[orphansIndex[i]][3] = numeric_limits<double>::infinity();
	}

}
