#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.hpp"
#include "algorithms.hpp"
bool enough = true;
/////////////////////////////////////////////////////////////////////////
nav_msgs::OccupancyGrid mapData;
nav_msgs::OccupancyGrid costmapData;
bool goalChanged = false;
double x_rob, y_rob, yaw_rob;
tf::Quaternion q;
double startThres = 0.5;
int tr = 600;
//////////////////////////////////////////////////////////////////////////
double x_goal, y_goal;
double x_start, y_start;
double step_size = 1; //for bigmap step is 1 for maze is 0.75
double robotNode = numeric_limits<double>::infinity();
bool reachedStart = false;
double val;
Row dist;
tuple<double, int> min_dist;

Row lmc;
vector<double> gValue;
N neighbors;
Row temp_newDist;
Matrix newDist;
Matrix Q;
double epsilon = 1;
vector<int> orphansIndex;
vector<double> closeNodesIndexStore;
bool noTree = false;
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
    //////////////////////////////////////GRAPH INIT////////////////////////////////////
    Row node = {x_goal, y_goal, 0, 0};
    Matrix graph;
    Matrix tree;

    graph.push_back(node);

    ////////////////////////////////////ROS INIT///////////////////////////////////////
    ros::init(argc, argv, "RRTx");
    ros::NodeHandle nh;
    ros::Subscriber costmapSub = nh.subscribe("/costmap_node/costmap/costmap", 100, costMapUpdateCallBack);
    ros::Subscriber mapSub = nh.subscribe("/map", 100, mapUpdateCallBack);
    ros::Subscriber rviz_sub = nh.subscribe("/move_base_simple/goal", 0, rvizGoalCallBack);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher poseStamped = nh.advertise<geometry_msgs::PoseStamped>("positionControl", 10);
    ros::Publisher robPose = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
    ros::Rate rate(1000);

    while (mapData.header.seq < 1 or mapData.data.size() < 1)
    {
        ROS_INFO("Marhale daryafte naghshe.");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    while (goalChanged == false)
    {
        ROS_INFO("Marhale dayafte hadaf az karbar.");
        ros::spinOnce();
        ros::Duration(.5).sleep();
    }

    //////////////////////////////////VISUALLLLLIZATION///////////////////////////////////////////////////
    //http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
    //line_strip is for continuous lines but line_list can connect 2 discontinuous points --> engari 2 ta 2 ta posht ham point haro mizari to list
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = costmapData.header.frame_id; //or "/map"
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
    line_strip.scale.x = 0.13;
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

    ///////////////////////////////////////////////SLAM POSE//////////////////////////////////////////////////////
    tf::TransformListener listener; //for getting the corrected pose
    tf::StampedTransform transform;
    int i = 1;
    while (ros::ok())
    {
        ///////////////////BALL//////////////////////////////
       // double r = shrinkingBallRadius(graph.size(), step_size);
        double r = step_size;
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

        x_rob = transform.getOrigin().x();
        y_rob = transform.getOrigin().y();
        q = transform.getRotation();
        yaw_rob = tf::getYaw(q);
        ////////////////PUB POSE//////////////////////////////
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x_rob;
        pose.pose.position.y = y_rob;
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rob);
        robPose.publish(pose);
        /////////////////GOAL CHANGED: RESET PLANNING//////////////////////////////
        if (goalChanged == true)
        {
            ROS_INFO("Reset plannnnnnnnnnnnnnning");
            i = 1;
            graph.clear();
            newDist.clear();
            Q.clear();
            lmc.clear();
            closeNodesIndexStore.clear();
            orphansIndex.clear();
            robotNode = numeric_limits<double>::infinity();
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
            points.points.clear();
            line_list.points.clear();
            line_strip.points.clear();
            geometry_msgs::Point p;
            p.x = x_goal;
            p.y = y_goal;
            p.z = 0;
            points.points.push_back(p);
            points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        }
        /////////////////////////UPDATE OBSTACLES!/////////////////////
        bool shouldUpdateObs = false;
        vector<double> closeNodesIndex;
        vector<double> diff;
        int segs = 2;
        if (i >= tr && fmod(i,10)==0)
        {
            for (int count = 1; count < graph.size(); count++)
            {

                // if (gridValue(costmapData, Row{graph[count][0], graph[count][1]}) > 0)
                //   closeNodesIndex.push_back(graph[count][2]);
                /*
                double parentIndex = graph[count][3];
                int check = costmapObstacleCheck(Row{graph[count][0], graph[count][1]}, Row{graph[parentIndex][0], graph[parentIndex][1]}, costmapData);

                if (check == 0)
                {
                    closeNodesIndex.push_back(graph[count][2]);
                    closeNodesIndex.push_back(graph[parentIndex][2]);
                }
                */
                double parentIndex = graph[count][3];
                int checkCur = gridValue(costmapData, Row{graph[count][0], graph[count][1]});
                int checkPar = gridValue(costmapData, Row{graph[parentIndex][0], graph[parentIndex][1]});
                int checkSeg;
                for (int j = 0; j < segs; j++)
                {
                    double x, y;
                    x = (j / segs) * (graph[count][0] + graph[parentIndex][0]);
                    y = (j / segs) * (graph[count][1] + graph[parentIndex][1]);
                    checkSeg = gridValue(costmapData, Row{x, y});
                    if (checkSeg > 0)
                        break;
                }
                if (checkCur > 0 || checkPar > 0 || checkSeg > 0)
                {
                    closeNodesIndex.push_back(graph[count][2]);
                    closeNodesIndex.push_back(graph[parentIndex][2]);
                    /* //rewire neighbor ham niaz be obstacleCkeck dare vase hamin goftam if else paeen ro pak kunam ta har dafe tamam orphan haro dobare dorost
                    //kune vali bi fayede bod! shayad bayad rewire ro dorost kunam ta obstacle check ham bokune moghe taghire parent!
                    vector<double>::iterator ip;
                    ip = std::unique(closeNodesIndex.begin(), closeNodesIndex.end());
                    closeNodesIndex.resize(std::distance(closeNodesIndex.begin(), ip));
                    if (closeNodesIndexStore.empty())
                        shouldUpdateObs = true;
                    */
                }
            }
            if (closeNodesIndexStore.empty())
            {
                vector<double>::iterator ip;
                ip = std::unique(closeNodesIndex.begin(), closeNodesIndex.end());
                closeNodesIndex.resize(std::distance(closeNodesIndex.begin(), ip));

                closeNodesIndexStore = closeNodesIndex;
                shouldUpdateObs = true;
            }
            else
            {
                vector<double>::iterator ip;
                ip = std::unique(closeNodesIndex.begin(), closeNodesIndex.end());
                closeNodesIndex.resize(std::distance(closeNodesIndex.begin(), ip));

                std::set_difference(closeNodesIndex.begin(), closeNodesIndex.end(), closeNodesIndexStore.begin(), closeNodesIndexStore.end(),
                                    std::inserter(diff, diff.begin()));
                closeNodesIndexStore = closeNodesIndex;
                closeNodesIndex = diff;
                if (!closeNodesIndex.empty())
                    shouldUpdateObs = true;
            }

            if (shouldUpdateObs == true)
                updateObstacles(costmapData, graph, Q, neighbors, newDist, gValue, lmc, orphansIndex, r, epsilon, robotNode, closeNodesIndex);
        }
        if (i < tr)
        {
            ////////////////////SAMPLING/////////////////////////

            ////BigMap param /////
            //double sample_x = randomGenerator(-2, 20);
            //double sample_y = randomGenerator(-2, 6);

            ////maze param /////
            double sample_x = randomGenerator(0, 9);
            double sample_y = randomGenerator(-1, 9);
            //double sample_x = randomGenerator(-2, 30);
            //double sample_y = randomGenerator(-2, 30);
            double random = randomGenerator(0, 1);

            if (random < 0.05 && reachedStart == false)
                node = {x_start, y_start, (double)i, 0};
            else
                node = {sample_x, sample_y, (double)i, 0};

            /////////////////////////////////BUILDING THE UNSATURATED GRAPH//////////////////////////////
            graph.push_back(node);
            /////////////////////////////////FINDING THE NEAREST NODE////////////////////////////////////
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

            //////////////////////////////////SATURATION////////////////////////////////////////////////
            if (min_dist_value > step_size)
            {
                saturate(graph[i], nearest_node, step_size);
            }
            //////////////////////////////////EXTEND////////////////////////////////////////////////////
            temp_newDist = extend(graph, r, lmc, neighbors, costmapData);
            newDist.resize(i);
            newDist.push_back(temp_newDist);

            if (graph.size() - 1 == i)
            {
                gValue.push_back(numeric_limits<double>::infinity());

                if (gValue[i] - lmc[i] > epsilon)
                {
                    verrifyQueue(Q, gValue[graph.size() - 1], lmc[graph.size() - 1], graph.size() - 1);
                    rewireNeighbors(graph.size() - 1, graph, Q, neighbors, newDist, gValue, lmc, r, epsilon, costmapData);
                }
                reduceInconsistency(Q, graph, neighbors, newDist, gValue, lmc, robotNode, r, epsilon, orphansIndex, costmapData);
                /////////////////////////////////REACHED THE START////////////////////////////////
                if (euc_dist(Row{graph[i][0], graph[i][1]}, Row{x_start, y_start}) < startThres && enough == true)
                {
                    reachedStart = true;
                    robotNode = i;
                    enough = false;
                }
                if (reachedStart == true)
                {
                    //clear RVIZ
                    points.action = points.DELETEALL;
                    line_strip.action = line_strip.DELETEALL;
                    line_list.action = line_list.DELETEALL;
                    points.points.clear();
                    line_list.points.clear();
                    line_strip.points.clear();
                    geometry_msgs::Point p;
                    p.x = x_goal;
                    p.y = y_goal;
                    p.z = 0;
                    points.points.push_back(p);
                    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
                    double cond = graph[robotNode][2];
                    int counter = 0;
                    tree.clear();

                    /////////////////wWHOLE GRAPH VISUALIZATION//////////////////////////OPTIONAL///////
                    for (int k = 0; k < graph.size(); k++)
                    {
                        if (graph[k][3] != numeric_limits<double>::infinity())
                        {
                            geometry_msgs::Point p;
                            p.x = graph[k][0];
                            p.y = graph[k][1];
                            p.z = 0;
                            points.points.push_back(p);
                            double parent_index = graph[k][3];
                            line_list.points.push_back(p);
                            p.x = graph[parent_index][0];
                            p.y = graph[parent_index][1];
                            p.z = 0;
                            line_list.points.push_back(p);
                        }
                    }
                    marker_pub.publish(line_list);
                    ////////////////////TREE CONSTRUCTION///////////////////AND VISUALIZATION///////////////////////////
                    noTree = false;
                    while (cond > 0)
                    {
                        if (graph[cond][3] == numeric_limits<double>::infinity())
                        {
                            ROS_WARN("NO TREE");
                            noTree = true;
                            break;
                        }
                        tree.push_back(Row{graph[cond][0], graph[cond][1], graph[cond][2], graph[cond][3]});

                        p.x = tree[counter][0];
                        p.y = tree[counter][1];
                        p.z = 0;
                        points.points.push_back(p);
                        line_strip.points.push_back(p);
                        cond = graph[cond][3];
                        counter++;
                        //ros::Duration(0.09).sleep();
                    }
                    if (noTree == false)
                    {
                        tree.push_back(Row{x_goal, y_goal});
                        p.x = x_goal;
                        p.y = y_goal;
                        p.z = 0;
                        line_strip.points.push_back(p);
                    }
                    else
                    {
                        line_strip.points.clear();
                    }
                    marker_pub.publish(line_strip); //addition
                    marker_pub.publish(points);
                }
                i++;
            }
        }

        if (i >= tr)
        {
            if (reachedStart == true)
            {
                /////////////////////////WHAT IS THE CURRENT ROBOT NODE//////////////////////////
                if (noTree == false)
                {
                    dist.clear();
                    for (int j = 0; j < tree.size(); j++)
                    {
                        //if (graph[j][3] != numeric_limits<double>::infinity())
                        //{
                        Row check_node = {tree[j][0], tree[j][1]};
                        Row current_node = {x_rob, y_rob};
                        val = euc_dist(check_node, current_node);
                        dist.push_back(val);
                        //}
                    }
                    min_dist = LargestOrSmallestElement(dist, dist.size(), "smallest");

                    double min_dist_value = get<0>(min_dist);
                    int min_dist_index = get<1>(min_dist);
                    robotNode = tree[min_dist_index][2];
                }
                else
                {
                    dist.clear();
                    for (int j = 0; j < graph.size(); j++)
                    {
                        //if (graph[j][3] != numeric_limits<double>::infinity())
                        //{
                        Row check_node = {graph[j][0], graph[j][1]};
                        Row current_node = {x_rob, y_rob};
                        val = euc_dist(check_node, current_node);
                        dist.push_back(val);
                        //}
                    }
                    min_dist = LargestOrSmallestElement(dist, dist.size(), "smallest");

                    double min_dist_value = get<0>(min_dist);
                    int min_dist_index = get<1>(min_dist);
                    robotNode = graph[min_dist_index][2];
                }

                //////////////////////////////////////////////////////////////////////////////////
                //clear RVIZ
                points.action = points.DELETEALL;
                line_strip.action = line_strip.DELETEALL;
                line_list.action = line_list.DELETEALL;
                points.points.clear();
                line_list.points.clear();
                line_strip.points.clear();
                geometry_msgs::Point p;
                p.x = x_goal;
                p.y = y_goal;
                p.z = 0;
                points.points.push_back(p);
                points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
                double cond = graph[robotNode][2];

                int counter = 0;
                tree.clear();

                /////////////////WHOLE GRAPH VISUALIZATION//////////////////////////OPTIONAL///////
                for (int k = 0; k < graph.size(); k++)
                {
                    if (graph[k][3] != numeric_limits<double>::infinity())
                    {
                        geometry_msgs::Point p;
                        p.x = graph[k][0];
                        p.y = graph[k][1];
                        p.z = 0;
                        points.points.push_back(p);

                        double parent_index = graph[k][3];
                        line_list.points.push_back(p);
                        p.x = graph[parent_index][0];
                        p.y = graph[parent_index][1];
                        p.z = 0;
                        line_list.points.push_back(p);
                    }
                }
                marker_pub.publish(line_list);
                ////////////////////TREE CONSTRUCTION///////////////////AND VISUALIZATION///////////////////////////
                noTree = false;
                while (cond > 0)
                {
                    if (graph[cond][3] == numeric_limits<double>::infinity())
                    {
                        ROS_WARN("NO TREE");
                        noTree = true;
                        break;
                    }
                    tree.push_back(Row{graph[cond][0], graph[cond][1], graph[cond][2], graph[cond][3]});
                    //RVIZ
                    p.x = tree[counter][0];
                    p.y = tree[counter][1];
                    p.z = 0;
                    points.points.push_back(p);
                    line_strip.points.push_back(p);
                    cond = graph[cond][3];
                    counter++;
                    //ros::Duration(0.09).sleep();
                }
                if (noTree == false)
                {
                    tree.push_back(Row{x_goal, y_goal});
                    p.x = x_goal;
                    p.y = y_goal;
                    p.z = 0;
                    line_strip.points.push_back(p);
                }
                else
                {

                    line_strip.points.clear();
                }
                marker_pub.publish(line_strip); //addition
                marker_pub.publish(points);

                /////////////////////////////////PUBLISHING POSE////////////////////////////////////////
                if (noTree == false)
                {
                    Row andix;
                    andix = find(tree, robotNode);
                    int par = tree[andix[0]][3];
                    if (euc_dist(Row{x_rob,y_rob},Row{x_goal,y_goal})<0.7)
                    {
                        geometry_msgs::PoseStamped point;
                        point.pose.position.x = x_rob;
                        point.pose.position.y = y_rob;
                        point.pose.position.z = 0;
                        point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rob);
                        poseStamped.publish(point); //publishing the currrent position
                    }
                    else
                    {
                        ////////////////////////////////////////////////////////
                        geometry_msgs::PoseStamped point;
                        point.pose.position.x = graph[par][0];
                        point.pose.position.y = graph[par][1];
                        point.pose.position.z = 0;
                        point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rob);
                        poseStamped.publish(point); //publish miniGoal
                    }
                }
                else
                {
                    geometry_msgs::PoseStamped point;
                    point.pose.position.x = x_rob;
                    point.pose.position.y = y_rob;
                    point.pose.position.z = 0;
                    point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rob);
                    poseStamped.publish(point); //publishing the currrent position
                }

                ////////////////////////////////////////////////////////////////////////////////////////////////////
            }
        }

        ROS_INFO_STREAM(i);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
