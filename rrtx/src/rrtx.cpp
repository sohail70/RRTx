//TODO : build the trajectory tree and plot it in rviz
//TODO : obstacles must be checked very carefully!
//TODO : May be I should reset planning when ever new map is recieved! what you say?!
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.hpp"
#include "algorithms.hpp"
//////////////////////////////////////////////////////////
nav_msgs::OccupancyGrid costmapData;
bool goalChanged = false;
//////////////////////////////////////////////////////////////////////////
double x_goal, y_goal;
double x_start, y_start;
double step_size = 0.5;
double robotNode = numeric_limits<double>::infinity();
bool reachedStart = false;
double val; //for finding nearest node
Row dist;   //for finding nearest node-->given to min function
tuple<double, int> min_dist;
//LMC initilization ...refrence it to the functions!
Row lmc;
// g_value initialization
vector<double> gValue;
// neighbor instance
N neighbors;
//there is NewDistance coming out of extend...maybe pass it by reference!
Row temp_newDist;
Matrix newDist;
//Initialization of priority queue
Matrix Q;
//epsilon consistent
double epsilon = 0.5;
//orphan nodes container
vector<int> orphansIndex;

/////////////////////////////////////////////////////////////////////////
void costMapUpdateCallBack(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg)
{
    costmapData = *costmap_msg;
}

void rvizGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    x_goal = msg->pose.position.x;
    y_goal = msg->pose.position.y;
    ROS_WARN("Hadafe jadid :  %f %f", x_goal, y_goal);
    goalChanged = true;
}

int main(int argc, char **argv)
{
    lmc.push_back(0);
    gValue.push_back(0);
    //graph initialization
    Row node = {x_goal, y_goal, 0, 0}; //a node in a tree //initializaton//forth column is yet to be valued!
    Matrix graph;                      //whole nodes --- graph it self---sotone sevom in index node hengame ijadesh hast//sotone chaharmo parent node hast
    graph.push_back(node);
}
