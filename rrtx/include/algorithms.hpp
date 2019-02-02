#pragma once
#include<iostream>
#include<tuple>
#include<vector>
#include<math.h>
#include"functions.hpp"


using namespace std;
//for shrinking ball radius
double shrinkingBallRadius(int number_of_nodes,double step_size);


//for saturate.cpp
void saturate(vector<double> &unsat_node, vector<double> nearest_node, double step_size);


//obstacle set-->it depends on the detecter that you use!-->currently it's circles and linSeg's 
struct obstacle
{
	vector<vector<double>> circles; //state: <x_center,y_center,radius>
	vector<vector<double>> lineSegs;//state: <x_center,y_center,length,angle>
};


//for extend
struct N; //forward dec
Row extend(Matrix& graph, double r, Row& lmc,N& neighbors,nav_msgs::OccupancyGrid mapsub);

//for findParent
Row findParent(Row current_node, Matrix nearest_nodes, Row dist_to_near_nodes, Row& lmc, vector<int> andix,int current_node_index,nav_msgs::OccupancyGrid mapsub);

// for neighbors of the current node: it is a matrix because every row is a specific node in which you store the four kind of neghbors
struct N
{
	vector<vector<int>> original_plus;
	vector<vector<int>> original_minus;

	vector<vector<int>> running_plus;
	vector<vector<int>> running_minus;
};



//for verrify queue
void verrifyQueue(Matrix& Q , double gValue , double lmc , double node);

//for rewireNeighbors
void rewireNeighbors(int someNodeIndex, Matrix& graph, Matrix& Q, N& neighbors, Matrix newDist, Row gValue, Row& lmc, double r, double epsilon);

//for culling neighbors
void cullNeighbors(Matrix graph, double someNodeIndex, N& neighbors, double r, Matrix newDist);

//for reduceInconsistency
int reduceInconsistency(Matrix& Q, Matrix& graph, N& neighbors, Matrix newDist, Row& gValue, Row& lmc, double robotNode, double r, double epsilon, vector<int> orphansIndex);

//for updateLmc
void updateLmc(int someNodeIndex, Matrix& graph, Matrix newDist, N neighbors, Row& lmc, double r, vector<int> orphansIndex);


//for updateObstacles
void updateObstacles(obstacle& obsSet, Matrix& graph, Matrix& Q, N& neighbors, Matrix& newDist, Row gValue, Row& lmc, vector<int> orphansIndex, double r, double epsilon, double robotNode, double safetyFactor);

//for removing an obstacle
void removeObstacle(obstacle& obsSet, Matrix& graph, Matrix& Q, N& neighbors, Matrix& newDist, Row gValue, Row& lmc, double r, vector<int> orphansIndex, double safetyFactor);

//for adding an obstacle
void addNewObstacle(obstacle& obsSet, Matrix& graph, Matrix& Q, N& neighbors, Matrix& newDist, Row gValue, Row& lmc, double r, vector<int>& orphansIndex, double robotNode, double safetyFactor);

//for propagating to the childs
void propagateDescendants(Matrix& graph, Matrix& Q, N& neighbors, vector<int>& orphansIndex, Row& gValue, Row& lmc);

//for orphan nodes
void verrifyOrphan(double someNodeIndex, Matrix& Q, vector<int>& orphansIndex);

//is some node position in obstacle region?
bool obstacleCheck(Row position, obstacle obs, double safetyFactor);