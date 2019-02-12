#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>
#include "std_msgs/Header.h"
#include "stdint.h"
#include <vector>
using namespace std;
typedef vector<double> Row;
#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679 //yo hahahaha, kar bas daghigh bashe
double roll, pitch, yaw;                                                                                          //robots!
//double target_roll, target_pitch, target_yaw;                                                                     //target
tf::Quaternion q;
double maxW = 0.5;
double target[2]; //target pose (x/y)
double threshold;
double position[2]; //robots current pose
double distToTarget;
double deltaG;
double G = 0;
double velocity = 0.2;

ros::Publisher command;
ros::Subscriber current_pos;
void targetCallback(const geometry_msgs::PoseStamped &msg);
void setPose(const geometry_msgs::PoseStamped &msg);
void piToPi(double &angle);
void distAndGToTarget();
void constraint(double &G);
double euc_dist(Row p1, Row p2);
void setVelocity();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrtx_control");
    ros::start();
    ros::NodeHandle nh;

    ros::Rate rate(100);
    ros::Subscriber targetSub = nh.subscribe("/positionControl", 10, targetCallback);
    ros::Subscriber robPose = nh.subscribe("/pose", 100, setPose);
    command = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    int iterator = 0;
    tf::TransformListener listener1; //for getting the corrected pose
    tf::StampedTransform transform1;
    while (ros::ok())
    {

        geometry_msgs::Twist controlMsg;

        if (euc_dist(Row{position[0], position[1]}, Row{target[0], target[1]}) < 0.01) //age in threshold ziad bashe robot tick tick mikune to har hadaf!--->idea behtari nadaram felan vase inke sorAT avalie ro sef kunam va vaghti ham resid be maghsad sorAt sefr she!
        {
            controlMsg.linear.x = 0;
        }
        else
        {
            distAndGToTarget();
            G = G + deltaG;
            constraint(G);
            setVelocity();
            controlMsg.linear.x = velocity;
            controlMsg.angular.z = G;
        }

        command.publish(controlMsg);

        iterator++;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void targetCallback(const geometry_msgs::PoseStamped &msg)
{
    target[0] = (float)msg.pose.position.x;
    target[1] = (float)msg.pose.position.y;
}

void setPose(const geometry_msgs::PoseStamped &msg)
{
    position[0] = (float)msg.pose.position.x; //robots current position
    position[1] = (float)msg.pose.position.y;

    double qx = msg.pose.orientation.x;
    double qy = msg.pose.orientation.y;
    double qz = msg.pose.orientation.z;
    double qw = msg.pose.orientation.w;

    tf::Quaternion q{qx, qy, qz, qw};
    tf::Matrix3x3 m{q};

    m.getEulerYPR(yaw, roll, pitch);
}

void piToPi(double &angle)
{
    if (angle > 2 * PI || angle < -2 * PI)
        angle = fmod(angle, 2 * PI);
    if (angle > PI)
        angle = angle - 2 * PI;
    if (angle < -PI)
        angle = angle + 2 * PI;
}

void distAndGToTarget()
{
    distToTarget = sqrt(pow(position[0] - target[0], 2) + pow(position[1] - target[1], 2));
    double angle;
    if (euc_dist(Row{position[0], position[1]}, Row{target[0], target[1]}) < 0.01)
        angle = 0;
    else
        angle = atan2(target[1] - position[1], target[0] - position[0]);

    deltaG = angle - yaw - G;
    piToPi(deltaG);
}
void constraint(double &G)
{
    if (G > maxW)
        G = maxW;
    if (G < -maxW)
        G = -maxW;
}

double euc_dist(Row p1, Row p2)
{
    double dist;

    dist = sqrt(pow(p2[1] - p1[1], 2) + pow(p2[0] - p1[0], 2));

    return dist;
}

void setVelocity()
{
    if (abs(deltaG) > 0.2)
        velocity = 0.1;
    else
        velocity = 0.2;
}
