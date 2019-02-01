#include "ros/ros.h"
#include <vector>
#include <tuple>
#include <random>
#include <math.h>
#include "nav_msgs/OccupancyGrid.h"

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>


using namespace std;

typedef vector<vector< double>> Matrix;
typedef vector<double> Row;
///////////////

/////////////
//tamame algorithm haee ke paeen minvisam joda test mishe va kenareshoon "OK" neveshte mishe ya "Not_tested"

tuple<double, int> LargestOrSmallestElement(Row , int , string ); //OK


double randomGenerator(int, int); //OK

double euc_dist(Row p1, Row p2);//OK

Row find(Matrix matrix, int node_index); //ok I guess!---> no it's not--->age tedad bishtar az dota peyda kune chi?!!-->find2
Matrix find2(Matrix matrix, int node_index);
vector<int> findRow(Matrix matrix, int node_index);

void sortrows(Matrix& matrix, int col);//oK I guess!

int findInVec(vector<int> vec, int number);//oK I guess!