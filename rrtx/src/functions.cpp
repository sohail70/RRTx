#include "functions.hpp"
#include "algorithms.hpp"
#include <iostream>
#include <cstring>
#include <random>
#include <tuple>
#include <math.h>
#include <algorithm>
using namespace std;

tuple<double, int> LargestOrSmallestElement(Row array, int size, string tip)
{
	int index = 0;
	tuple<double, int> min_dist;
	double largest = array[0];
	double smallest = array[0];

	for (int i = 1; i < size; i++)
	{
		if (tip == "largest")
		{
			if (array[i] > array[index])
			{
				index = i;
				largest = array[i];
			}
		}
		else if (tip == "smallest")
		{
			if (array[i] < array[index])
			{
				index = i;
				smallest = array[i];
			}
		}
	}

	if (tip == "largest")
		min_dist = make_tuple(largest, index);
	else if (tip == "smallest")
		min_dist = make_tuple(smallest, index);

	return min_dist;
}

double randomGenerator(int range_from, int range_to)
{
	std::random_device rand_dev;
	std::mt19937 generator(rand_dev());
	std::uniform_real_distribution<double> distr(range_from, range_to);

	return distr(generator);
}

double euc_dist(Row p1, Row p2)
{
	double dist;
	dist = sqrt(pow(p2[1] - p1[1], 2) + pow(p2[0] - p1[0], 2));

	return dist;
}

Row find(Matrix matrix, int node_index)
{
	Row index;

	for (int i = 0; i < matrix.size(); i++)
		for (int j = 0; j < matrix[i].size(); j++)
			if (matrix[i][j] == node_index)
			{
				index.push_back(i);
				index.push_back(j);
			}

	if (!index.empty())
		return index;
	else
		return Row{-1, -1};
}

Matrix find2(Matrix matrix, int node_index)
{
	Row index;
	Matrix mat;
	for (int i = 0; i < matrix.size(); i++)
		for (int j = 0; j < matrix[i].size(); j++)
			if (matrix[i][j] == node_index)
			{
				index.push_back(i);
				index.push_back(j);
				mat.push_back(index);
				index.clear();
			}

	if (!mat.empty())
		return mat;
	else
	{
		mat.push_back({-1, -1});
		return mat;
	}
}

vector<int> findRow(Matrix matrix, int node_index)
{
	vector<int> index;
	for (int i = 0; i < matrix.size(); i++)
		for (int j = 2; j < matrix[i].size(); j++)
			if (matrix[i][j] == node_index)
			{
				index.push_back(i);
				break;
			}

	if (!index.empty())
		return index;
	else
	{
		return vector<int>{-1};
	}
}

int findInVec(vector<int> vec, int number)
{
	for (int i = 0; i < vec.size(); i++)
		if (vec[i] == number)
		{
			return i;
		}
	return -1;
}

void sortrows(Matrix &matrix, int col)
{
	sort(matrix.begin(),
		 matrix.end(),
		 [col](const vector<double> &lhs, const vector<double> &rhs) {
			 int size = 2;
			 int temp = col;
			 for (int i = 0; i < size; i++)
			 {
				 if (lhs[temp] == rhs[temp])
					 temp += 1;
			 }
			 return lhs[temp] < rhs[temp];
		 });
}

//Adopted funcs
int costmapObstacleCheck(Row xnear, Row xnew, nav_msgs::OccupancyGrid mapsub)
{
	double rez = double(mapsub.info.resolution) * 2;
	double stepz = int(ceil(euc_dist(xnew, xnear)) / rez);
	Row xnew_ = xnew;
	Row xnear_ = xnear;
	int obstacle = 0;
	int ret = 0;

	for (int c = 0; c < stepz; c++)
	{
		saturate(xnew_, xnear_, rez);
		xnear_ = xnew_;
		xnew_ = xnew;

		if (gridValue(mapsub, xnear_) > 0)
			obstacle = 1;
	}

	if (obstacle == 1)
		ret = 0;
	else
		ret = 1;

	return ret;
}

int gridValue(nav_msgs::OccupancyGrid &mapData, Row Xp)
{
	double resolution = mapData.info.resolution;
	double Xstartx = mapData.info.origin.position.x;
	double Xstarty = mapData.info.origin.position.y;
	double width = mapData.info.width;
	std::vector<signed char> Data = mapData.data;
	double indx = (floor((Xp[1] - Xstarty) / resolution) * width) + (floor((Xp[0] - Xstartx) / resolution));
	int out;
	out = Data[int(indx)];
	return out;
}