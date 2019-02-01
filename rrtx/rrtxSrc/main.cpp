/*
RRTx Reporting for duty 
Author: Sohail Espahbodi Nia --->sohe(a)il.e.nia@gmail.com


TODO : check neighbor sets in matlab-->but how? maybe after rewiring precedure --> plot the graph in matlab
	 : or maybe after everything is done! not sure :O)
TODO:robotNode bayad oon paeen set beshe !
	: reduceInconsistency heavy ast! conition haro doros kun!
	: cullneighbor karo kharab mikune -> doros she!


TODO: complete the UpdateObstacle
TODO: use IF statement before going to the extend function!--->think this through!-->it's important
	  for the new sampled node to not be in obstacle region (ignore those samples by using if statement!)

handy keyboard shortcuts: ctrl+m+h to create a collapsible region and ctrl+m+u to undo it
*/

#include <iostream>
//#include "design_param.h"
#include "magic_algorithms.h"
#include <tuple>
#include <vector>
#include "rrtx_algorithms.h"
#include <fstream>
#include <iterator>
using namespace std;




int main()
{
	double x_goal = 47;
	double y_goal = 47;

	double x_start = -47;
	double y_start = -47;
	Row start = { x_start,y_start };
	double robotNode = numeric_limits<double>::infinity();//once the tree reached the robot then you should change it appropriately
	
	
	bool reachedStart = false;
	double val; //for finding nearest node
	Row dist;//for finding nearest node-->given to min function

	tuple<double, int> min_dist; //for finding the minimum distance between alot of dist to fin the nearest node with it's index
	
	double step_size=8;

	//LMC initilization ...refrence it to the functions!
	Row lmc;
	lmc.push_back(0);
	// g_value initialization
	vector<double> gValue;
	gValue.push_back(0);

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

	//obstacle set
	obstacle obs;//in bayad biroon bashe ta beshe dar updateObstacle ham azash estefade kard!
	
	//safety factor for orphan nodes and obstacle check function
	double safetyFactor = 1.5;

	//graph initialization
	Row node = { x_goal,y_goal,0,0 };    //a node in a tree //initializaton//forth column is yet to be valued!
	Matrix graph; //whole nodes --- graph it self---sotone sevom in index node hengame ijadesh hast//sotone chaharmo parent node hast
	graph.push_back(node);
	

	//saving to a file
	ofstream output_file("graph.txt");
	ostream_iterator<double> output_iterator(output_file, "\n");
	
	int counter;
	int	 i = 1; //az yek chun i=0 init shode
	while (1) {

		//rrt star shrinking ball radius
		double r = shrinkingBallRadius(graph.size(), step_size);
		
		//rrt constant radius
		//double r = step_size+.1;


		//updating obstacle----> in khoobeh ke avale algorithm bashe ta dg niaz be cheking vasat code nabashe!--->kholase prose detect kardan obstacle va gozashtanesh 
		//to map tavasot SLAM anjam mishe ta(behtare ye node ba python benvisi vase SLAM) bad position obs-ha bedast miad
		//hala ba raveshi ke to rrtx dar pavaraghi gofte ba estefade az fasele robot ta obs mogheyate node haye khrab shode ro mishe bedast ovor va dar
		// propagate descendant emal kard

		//nokte: update upstacle faghat dar sesor radius etefagh miofte ke vase turtlebot 3 mishe 3.5 meters fek kunam
		
		
		bool isObstacleChanged = false; //how? use obstacleDetector constantly checking if any obstacles added to the rviz
		//oon simulation eee ke man didam hamash dar hale taghiire, ye seri circle dare ezafe mishe va hazf mishe!
		//bayad avalan focus kunim to mahdodeye dide robotemoon va  taghiiirate obstacle ro onja mad nazar gharar bedim
		//state ghabli obs ha ba state feli obs ha check mishe!-->like data association!
		// obstacle state ro chi begirim? maybe! <x_center,y_center> if it's a circle!---> should I add velocity state to the moving circles? making it little bit harder? let me think..
		// and if it's a line! we can use <x_center,y_center,lenght/2, angle!> I guess! or may be (radius,theta,lenght!) as in: 
		//https://en.wikipedia.org/wiki/Hough_transform#Theory
		//kholase farz ro bar in bezar ke betouni shart isObstacleChanged ro befahmi!
		


		//vase debug ye test anjam midim:--->tashkhis in dar amal khodesh dastan dare!-->bala tozih dadam!
		
		isObstacleChanged = false;
		if (i == 5000) //deghat kun i>18 bashe chun sample hamash az start node hast ta in i va obstacle bezari ghabl az in dastan mojebe dore batel mishe!
		{
			Row circle1 = { 0,0,10 };//I make obstacles!--->ino bayad node detection behemoon bede!
			
			obs.circles.push_back(circle1);
			
			isObstacleChanged = true;
		}
		if (i == 6000) //deghat kun i>18 bashe chun sample hamash az start node hast ta in i va obstacle bezari ghabl az in dastan mojebe dore batel mishe!
		{
			Row circle1 = { 20,20,10 };//I make obstacles!--->ino bayad node detection behemoon bede!

			obs.circles.push_back(circle1);

			isObstacleChanged = true;
		}if (i ==7000) //deghat kun i>18 bashe chun sample hamash az start node hast ta in i va obstacle bezari ghabl az in dastan mojebe dore batel mishe!
		{
			Row circle1 = { -30,-30,5 };//I make obstacles!--->ino bayad node detection behemoon bede!

			obs.circles.push_back(circle1);

			isObstacleChanged = true;
		}
		/*
		if (i == 4000)
		{
			Row circle1 = { -5,7,5 };//I make obstacles!--->ino bayad node detection behemoon bede!

			obs.circles.push_back(circle1);

			isObstacleChanged = true;
		}
		*/
		if (i == 20000)
		{
			cout << "removing obs" << endl;
			isObstacleChanged = true;
		}

		if (isObstacleChanged == true) //three cases: obs added/obs removed/ old obstacle in a set moved to a new location which I guess you can discrete
			//that into removed obs and then add obs!(I guess!)--->adding/removing shoud be discretized I guess
		{
			updateObstacles(obs, graph, Q, neighbors, newDist, gValue, lmc, orphansIndex, r, epsilon, robotNode, safetyFactor);
		}


		



		/*
		TODO: sensor data must be evaluated to detect the obstacle
		
		if (isObstacle == true)
		{
			add or remove obstacle --> maybe estimate it by circle containing the obstacle
			propagate descendants 
			verrify queue
			reduceInconsistency
		}
		*/



		//generating 2 random number
		double sample_x = randomGenerator(-47, 47);
		double sample_y = randomGenerator(-47, 47);
		//probability: felan chun mohit static nemikham bezaram in mohem nist vali mizaram ta badan age naghshe static ham dashtim azash estefade kunam
		double rand = randomGenerator(0, 1);
		//halghe aval mostaghim mire goal ro be start vasl mikune va halghe dovum badesh fa@al mishe!
		if (rand > 0 && reachedStart == false)
		{
			node = { x_start,y_start,(double)i,0 };
		}
		else
		{
			node = { sample_x,sample_y,(double)i,0 };
		}
		
		
		//building the unsaturated graph
		graph.push_back(node);
		
		//finding the nearest node to the current node in order to saturate the node and build the saturated graph
		dist.clear();
		for (int j = 0; j < i ; j++)
		{
			Row check_node = { graph[j][0],graph[j][1] };
			Row current_node = { graph[i][0],graph[i][1] };
			val = euc_dist(check_node, current_node);
			dist.push_back(val);
		}
		min_dist = LargestOrSmallestElement(dist, dist.size(), "smallest");

		double min_dist_value = get<0>(min_dist); //nearest node distance to current node
		int min_dist_index = get<1>(min_dist); //nearest node index in the graph(index dar sotone sevom graph hast)
		Row nearest_node = { graph[min_dist_index][0],graph[min_dist_index][1] }; //this is the x ,y position of the nearest node


		//saturate the node
		if (min_dist_value > step_size)
		{
			saturate(graph[i], nearest_node,step_size);
		}
		//cout << euc_dist(graph[i], graph[i - 1]); ---->for debangiing purpose---it must be equal to step_size for the first few iteration!
		
		
		//TODO: when v is not in obstacle region, then extend, else don't ---> when you Implemented the UpdateObstacle make sure to do it
		//TODO: check the saturated line with points for robustness!

		//extend ing
		temp_newDist=extend(graph, r, lmc, neighbors,obs,safetyFactor);
		


		//storing newDistances
		//https://www.quora.com/Is-it-possible-to-create-an-array-vector-that-contains-variable-size-arrays-in-C++
		//hint: node aval ke ghabl az while ast(goal node vase i=0 hast)...newDist[0][0]=0 hast...newDist[1][0]-->faseleye node 1 az sefr ya 1 az sefr,,,,node[2][0]-->faseleye node 2 az node 0 "ya" node 0 az node 2;
		newDist.resize(i); //in khoobeh...node aval ke kharej az while ham hast be sefr set mikune...
		newDist.push_back(temp_newDist);

		


		//Checker for new sampling nodes inorder to ---> I guess I don't need it if updateObstacle implemented properly
		
		//verrify queue---> no need for this--> I don't know why did I put this function here in my MATLAB code!--> I don't remember the reason!

		
		
		
		if (graph.size() - 1 == i)//if the current node added to the tree then count it up//momkene node feli to r nabashe va ma deletesh kunim to extend
		{
			//rewire neighbors
			gValue.push_back(numeric_limits<double>::infinity());
			if (gValue.size() <= i || lmc.size() <= i)
				cout << "what";
			if (gValue[i] - lmc[i] > epsilon)
			{
				//if vIsObstacleFree
				verrifyQueue(Q, gValue[graph.size() - 1], lmc[graph.size() - 1], graph.size() - 1);

				rewireNeighbors(graph.size() - 1, graph, Q, neighbors, newDist, gValue, lmc, r, epsilon); //why did i used graph.size() instead of i? because graph ending might be deleted but i counter is not updated yet!
			}


			//reduce Inconsistency
			counter=reduceInconsistency(Q, graph, neighbors, newDist, gValue, lmc, robotNode, r, epsilon, orphansIndex);


			//for shifting the sampling phase: deghat kun age obstacle static pash biad vasat nabayad mostaghim vasl kuni va loope zir ham bayad hazf kuni--> kholase sample hato dorost kun age naghshe static dar dastras bud!
			if (reachedStart == false)
			{
				double measure = euc_dist({ graph[i][0],graph[i][1] }, start);
				if (measure < 0.5)
				{
					reachedStart = true;
					robotNode = i; //robot's current node---> tree reached the current robots postion -->TODO: but don't let it move yet! let the graph get reacher and 
					//then let the robot move, in order to move, you need to set the local goal for the robot(check for the robots node parent for the local goal)
					// when it reaches it's parent, you need to change the robot node to the current node(use some threshold for convenience)
				}
			}
			//Post Processing--> counting up the iterator - displaying things for debug 
			//cout << i<<endl;// << "  " << newDist.size() << endl;   //debug
			cout << i << "    " << counter << "     " << r << endl;// << graph[i - 1][0] << "    " << graph[i - 1][1] << "   " << graph[i - 1][3] << endl; // this the rrt tree--- not rrt star!

			i += 1;
		}

		
		if (i == 3000)
		{
			//copy every row of a graph into txt file
			for(int j=0;j<graph.size();j++)
				copy(graph[j].begin(), graph[j].end(), output_iterator);
			break;
		}

	}
	cout << "The end";
	cin.get();
}

