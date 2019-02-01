#pragma once

#include<iostream>
/*
TODO: I didn't transfer the plot var's ...just remeber to optimize the code...most of the variables aren't neccessary
*/
using namespace std;
//starting point
float source[2] = { -47,-47 }; //jaye feli robot dar launch file moshakhas mishe va dar ros bayad havaset bashe!

//end point of the tree
float goal[2] = { 47,47 };

//goal threshold: ye dayere vase goal dar nazar migiram ke vaghti robot resid behesh vayste: albate in vase ROS hast va felan ke nemishe to c++ robot ro harkat bedam ya controller bezaram!
int goal_thr = 4;

// fasele ro pre allocate mikunam-->ino bayad be forme matrix ee benvisi vali inja pointer be pointer minvisam ta 2D beshe

//ex:  int a[4] = { 10,11,12,13 };
//ex:  int b[2] = { 1,2 };
//TODO: delete vase new paeen yadet nare
int** dist = new int* [500]; //500 shayad kafi nabashe---badan resize she

//ex: *dist = a; --->yani radife aval 4 ta element dashte bashe
//ex: *(dist + 1) = b; --->radife dovom 2 ta element dashte bashe


//moteghayerhaye paeen felan belas estefade hastan....vase ROS bayad modify beshan

//controller cte

double dt = 0.05; //sample time
double vel_min = 1; //min velocity of robot
double vel_max = 8; //max velocity of the robot
double acc_min = -10;
double acc_max = 10;

double vel = vel_max; //current velocity used for the fisrt time

double w = 0.05; //I guess this is the first omega used for robot
double w_max = 1.2;

double kp = 5; //PID
double ki = 0; //PID
double kd = 1; //PID

double* error = new double[500]; 
//TODO: *error=0; ro natonesti bezari ...ino faghat mishe dakhele namespace ya main gozasht fek kunam vagarna error mide!

bool reached = false; //Query the robot if it's reached the goal or not
double g = 1; //can't remember what is this
bool fisrt = true;//can't remember

//RRTx vars:



double v = 0; //can't remember
double i = 2; //can't remember //TODO: may be this is index or sth and you should change it to int

// you need this in extend func..but for now...don't do it
//garche fek kunam in allocation kumaki nakune bayad pointer be pointer (shayadam bishtar bayad bezari)
struct N {
	double* original_plus = new double[3]; //you should do this later: *(ob.original_plus)=10;
	double* original_minus = new double[3];
	double* running_minus = new double[3];
	double* running_plus = new double[3];
};


double* g_v = new double[500]; //this is important...this is g_value..maybe change the size in future
double epsilon = 0.1; //epsilon consistency
double** Q = new double* [500]; //this is Queue I guess ...it's n by 3 matrix in my algorithm...use this pointer materials wisely;