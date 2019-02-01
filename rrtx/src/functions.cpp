#include "functions.hpp"
#include"algorithms.hpp"
#include<iostream>
#include<cstring>
#include <random>
#include <tuple>
#include <math.h>
#include <algorithm>
using namespace std;



//ye araye midi andixe koochaktarin element ya bozogtarinesho behet mide!
//deghat kun andix az 0 shoro mishe!
//@PARAM tip:"smallest" or "largest"

/* rahnamee estefade az func paeen dar main:
			int main()
			{
				tuple<double, int> min;
				double a[5] = { 10,20,30,40,50 };
				min = LargestOrSmallestElement(a, 5, "largest");
				double val = get<0>(min);
				int index = get<1>(min);
				cout << val << endl;
				cout << index << endl;
				cin.get();
			}
*/
tuple<double, int> LargestOrSmallestElement(Row array, int size, string tip)
{
	int index = 0;
	tuple <double, int> min_dist;
	double largest=array[0]; //initialize for comparison
	double smallest=array[0];//initialize for comparison

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



// random number generator....UNIFORMLY!
//yek adad beine dota vorodish mide behet...vaghti ehtemale hamashoon yeksane pas vaghti doba sedash bezani ahamiati peyda nemikune
//pas negaran nabash ke araye behet nemide
double randomGenerator(int range_from, int range_to)
{
	std::random_device                  rand_dev;
	std::mt19937                        generator(rand_dev());
	std::uniform_real_distribution<double>  distr(range_from, range_to);
	
	return distr(generator);
}



//distance between 2 points

double euc_dist(Row p1, Row p2) //size is obvious -->euclidean distance
{
	double dist;
	dist = sqrt(pow(p2[1] - p1[1], 2) + pow(p2[0] - p1[0], 2));
	
	return dist;
}


//let's find row and column of a specified number in a matrix
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
	else //not found
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
	else //not found
	{
		mat.push_back({ -1,-1 });
		return mat;
	}
}

vector<int> findRow(Matrix matrix, int node_index)
{
	vector<int> index;
	for (int i = 0; i < matrix.size(); i++)
		for (int j = 2; j < matrix[i].size(); j++) //deghat kun az sotone sevom shoro kardam chun sotone 1 va 2 bug ijad mikune-->kulan in algorithm specific vase yekare khase!
			if (matrix[i][j] == node_index)
			{
				index.push_back(i);
				break; //for avoiding duplication
			}

	if (!index.empty())
		return index;
	else //not found
	{
		return vector<int>{ -1 };
	}
}


//let's find specific element in a Row (vector) and return the index in a vector
int findInVec(vector<int> vec,int number)
{
	for (int i = 0; i < vec.size(); i++)
			if (vec[i] == number)
			{
				return i;
			}
		return -1;
}







// this is my version of sortrow

void sortrows(Matrix& matrix, int col) {
	sort(matrix.begin(),
		matrix.end(),
		[col](const vector<double>& lhs, const vector<double>& rhs) { //third argument of sort is "HOW TO SORT IT?" increasing or decreasing or any lambda!
		int size = 2;//how many columns does your matrix have?I have 3 column but the first two is important for the computation
		int temp = col;
		for (int i = 0; i <size; i++)//check all columns
		{
			if (lhs[temp] == rhs[temp]) // if the top element is equal to bottom element
				temp += 1;//use the next column for the sort function
		}
		return lhs[temp] < rhs[temp];
	});
}





int costmapObstacleCheck(Row xnear, Row xnew, nav_msgs::OccupancyGrid mapsub)
{
	double rez = double(mapsub.info.resolution) * 1;
	double stepz = int(ceil(euc_dist(xnew, xnear)) / rez);
	Row xnew_ = xnew;
	Row xnear_ = xnear;
	int obs = 0;
	int out = 0;

	for (int c = 0; c < stepz; c++)
	{
		saturate(xnew_, xnear_, rez);
		xnear_ = xnew_; 
		xnew_ = xnew; //reset
		//ROS_WARN("check %i",gridValue(mapsub, xnear_));
		if (gridValue(mapsub, xnear_) > 0) //costMap grid value is between 96-99 -->
			obs = 1;
	}

	if (obs == 1)
		out = 0; //obstacle
	else
		out = 1; //no obstacle

	return out;
}

//gridValue function
int gridValue(nav_msgs::OccupancyGrid &mapData, Row Xp)
{

	double resolution = mapData.info.resolution;
	double Xstartx = mapData.info.origin.position.x;
	double Xstarty = mapData.info.origin.position.y;

	double width = mapData.info.width;
	std::vector<signed char> Data = mapData.data;

	//map data:  100 occupied      -1 unknown       0 free

	double indx = (floor((Xp[1] - Xstarty) / resolution) * width) + (floor((Xp[0] - Xstartx) / resolution));
	int out;
	out = Data[int(indx)]; 
	return out;
}