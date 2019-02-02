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
bool enough = true;
//////////////////////////////////////////////////////////
nav_msgs::OccupancyGrid mapData;
nav_msgs::OccupancyGrid costmapData;
bool goalChanged = false;
double x_rob, y_rob, yaw_rob;
tf::Quaternion q;        //robots current quat
double startThres = 0.1; //if it's too low the rrt keeps planning which sucks!

//////////////////////////////////////////////////////////////////////////
double x_goal = 10, y_goal = 10;
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
double epsilon = 0.05;
//orphan nodes container
vector<int> orphansIndex;

/////////////////////////////////////////////////////////////////////////
void mapUpdateCallBack(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    mapData = *map_msg;
}

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
    Matrix tree;                       //shortest tree!

    graph.push_back(node);

    /////////////////////////////////////
    ros::init(argc, argv, "RRTx");
    ros::NodeHandle nh;
    ros::Subscriber costmapSub = nh.subscribe("/costmap_node/costmap/costmap", 100, costMapUpdateCallBack);
    ros::Subscriber mapSub = nh.subscribe("/map", 100, mapUpdateCallBack);
    ros::Subscriber rviz_sub = nh.subscribe("/move_base_simple/goal", 0, rvizGoalCallBack);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate rate(600);

    while (mapData.header.seq < 1 or mapData.data.size() < 1)
    {
        ROS_WARN("marhale daryafte naghshe!");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    while (goalChanged == false)
    {
        ROS_WARN("waiting");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    //////////////////////////////////VISUALLLLLIZATION///////////////////////////////////////////////////
    ROS_WARN("flag 1");
    //http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
    //line_strip is for continuous lines but line_list can connect 2 discontinuous points --> engari 2 ta 2 ta posht ham point haro mizari to list
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = mapData.header.frame_id; //or "/map"
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.08;
    points.scale.y = 0.08;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.08;
    line_list.scale.x = 0.05;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0f;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.color.r = 0.0;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    /////////////////VISUALAIZING THE FIRST NODE////////////////
    geometry_msgs::Point p;
    p.x = x_start;
    p.y = y_start;
    p.z = 0;
    points.points.push_back(p);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    tf::TransformListener listener; //for getting the corrected pose
    tf::StampedTransform transform;
    int i = 1;
    while (ros::ok())
    {
        ///////////////////BALL//////////////////////////////
        //double r = shrinkingBallRadius(graph.size(), step_size);
        double r = 1;
        ///////////////////CURRENT POSE//////////////////////

        int temp = 0;
        while (temp == 0)
        {
            try
            {
                temp = 1;
                listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                temp = 0;
                ros::Duration(0.1).sleep();
            }
        }

        x_rob = transform.getOrigin().x(); //robots current position --> Corrected position from SLAM
        y_rob = transform.getOrigin().y();
        q = transform.getRotation();
        yaw_rob = tf::getYaw(q);
        ////////////////PUB POSE//////////////////////////////

        /////////////////GOAL CHANGED: RESET PLANNING//////////////////////////////
        if (goalChanged == true)
        {
            ROS_WARN("Reset plannnnnnnnnnnnnnning");
            i = 1;
            graph.clear();
            newDist.clear();
            Q.clear();
            lmc.clear();
            gValue.clear();
            lmc.push_back(0);
            gValue.push_back(0);
            neighbors.original_minus.clear();
            neighbors.original_plus.clear();
            neighbors.running_minus.clear();
            neighbors.running_plus.clear();
            x_start = x_rob;
            y_start = y_rob;

            node = {x_goal, y_goal, 0.0, 0.0};
            graph.push_back(node);

            goalChanged = false;
            reachedStart = false;
            enough = true;
            points.action = points.DELETEALL;
            line_strip.action = line_strip.DELETEALL;
            line_list.action = line_list.DELETEALL;
            points.points.clear(); //in khat miheme ke point haye ghabli ro pas az set shodan goal jadid pak mikune!
            line_list.points.clear();
            line_strip.points.clear();
            geometry_msgs::Point p;
            p.x = x_goal;
            p.y = y_goal;
            p.z = 0;
            points.points.push_back(p);
            points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        }
        ////////////////////SAMPLING/////////////////////////
        double sample_x = randomGenerator(-2, 20);
        double sample_y = randomGenerator(-2, 6);
        double random = randomGenerator(0, 1);

        if (random < 0.12 && reachedStart == false)
            node = {x_start, y_start, (double)i, 0};
        else
            node = {sample_x, sample_y, (double)i, 0};

        ///////////////building the unsaturated graph////////////////
        graph.push_back(node);
        ///////////////////////finding the nearest node to the current node in order to saturate the node and build the saturated graph
        dist.clear();
        for (int j = 0; j < i; j++)
        {
            Row check_node = {graph[j][0], graph[j][1]};
            Row current_node = {graph[i][0], graph[i][1]};
            val = euc_dist(check_node, current_node);
            dist.push_back(val);
        }
        min_dist = LargestOrSmallestElement(dist, dist.size(), "smallest");

        double min_dist_value = get<0>(min_dist);
        int min_dist_index = get<1>(min_dist);
        Row nearest_node = {graph[min_dist_index][0], graph[min_dist_index][1]};

        //saturate the node
        if (min_dist_value > step_size)
        {
            saturate(graph[i], nearest_node, step_size);
        }
        //ROS_WARN("flag 1");
        temp_newDist = extend(graph, r, lmc, neighbors, costmapData);
        newDist.resize(i);
        newDist.push_back(temp_newDist);

        if (graph.size() - 1 == i) //if the current node added to the tree then count it up//momkene node feli to r nabashe va ma deletesh kunim to extend
        {
            //ROS_WARN("flag 2");
            gValue.push_back(numeric_limits<double>::infinity());

            if (gValue[i] - lmc[i] > epsilon)
            {
                //if vIsObstacleFree
                verrifyQueue(Q, gValue[graph.size() - 1], lmc[graph.size() - 1], graph.size() - 1);

                rewireNeighbors(graph.size() - 1, graph, Q, neighbors, newDist, gValue, lmc, r, epsilon); //why did i used graph.size() instead of i? because graph ending might be deleted but i counter is not updated yet!
            }
            //ROS_WARN("flag 3");
            //ROS_WARN("ssssssssssss %f %f %f %f",graph[i][0],graph[i][1],graph[i][2],graph[i][3]);

            reduceInconsistency(Q, graph, neighbors, newDist, gValue, lmc, robotNode, r, epsilon, orphansIndex);
            double startIndex;
            /////////////////////////////////REACHED THE START////////////////////////////////
            if (euc_dist(Row{graph[i][0], graph[i][1]}, Row{x_start, y_start}) < startThres && enough == true)
            {
                reachedStart = true;
                startIndex = i;
                enough = false;
            }
            if (reachedStart == true)
            {
                //clear RVIZ
                points.action = points.DELETEALL;
                line_strip.action = line_strip.DELETEALL;
                line_list.action = line_list.DELETEALL;
                points.points.clear(); //in khat miheme ke point haye ghabli ro pas az set shodan goal jadid pak mikune!
                line_list.points.clear();
                line_strip.points.clear();
                geometry_msgs::Point p;
                p.x = x_goal;
                p.y = y_goal;
                p.z = 0;
                points.points.push_back(p);
                points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
                //////

                //double value = euc_dist(Row{graph[i][0], graph[i][1]}, Row{x_start, y_start});
                //ROS_WARN("value: %f", value);

                ROS_WARN("start reacheddddddddddddd");
                double cond = graph[startIndex][2];
                ROS_WARN("graph last index: %f", cond);
                int counter = 0;
                //geometry_msgs::Point p;
                tree.clear();
                /*
                for (int k = 0; k < graph.size(); k++)
                {
                    ROS_WARN("graph[%i]  %f %f %f %f %f", k, graph[k][0], graph[k][1], graph[k][2], graph[k][3], lmc[k]);
                }
                */
                while (cond > 0)
                {
                    tree.push_back(Row{graph[cond][0], graph[cond][1]});
                    //ROS_WARN("graph: %f %f %f %f", graph[cond][0], graph[cond][1], graph[cond][2], graph[cond][3]);
                    //RVIZ
                    p.x = tree[counter][0];
                    p.y = tree[counter][1];
                    p.z = 0;
                    points.points.push_back(p);
                    line_strip.points.push_back(p);
                    cond = graph[cond][3];
                    counter++;
                    //ROS_WARN("flag 1");
                    //ros::Duration(0.09).sleep();
                }
                tree.push_back(Row{x_goal, y_goal});
                p.x = x_goal;
                p.y = y_goal;
                p.z = 0;
                line_strip.points.push_back(p);
                marker_pub.publish(line_strip); //addition
                marker_pub.publish(points);
            }
            /*
            while (reachedStart == true)
            {
                if (euc_dist(Row{graph[i][0], graph[i][1]}, Row{x_start, y_start}) < startThres)
                {
                    reachedStart = true;
                    for (int k = 0; k < tree.size(); k++)
                        ROS_WARN("tree:  %f %f ", tree[k][0], tree[k][1]);
                    if (goalChanged == true)
                        break;
                }
                else
                {
                    reachedStart = false;
                }
                ros::spinOnce();
                ros::Duration(1).sleep();
            }
            */
            i++;
        }

        ROS_WARN("iter %i", i);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
