#pragma once
#include<iostream>
#include<cstring>
#include<vector>
using namespace std;

typedef vector< vector<double>> Matrix;
typedef vector<double> Row;

//tamame algorithm haee ke paeen minvisam joda test mishe va kenareshoon "OK" neveshte mishe ya "Not_tested"

tuple<double, int> LargestOrSmallestElement(Row , int , string ); //OK


double randomGenerator(int, int); //OK

double euc_dist(Row p1, Row p2);//OK

Row find(Matrix matrix, int node_index); //ok I guess!---> no it's not--->age tedad bishtar az dota peyda kune chi?!!-->find2
Matrix find2(Matrix matrix, int node_index);
vector<int> findRow(Matrix matrix, int node_index);

void sortrows(Matrix& matrix, int col);//oK I guess!

int findInVec(vector<int> vec, int number);//oK I guess!