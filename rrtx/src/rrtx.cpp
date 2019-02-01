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

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>








int main(int argc, char **argv)
{
	ros::Rate rate(100);
    ros::init(argc, argv, "optimized_rapid_random_tree");

        ros::spinOnce();
        rate.sleep();
    
    return 0;
}
