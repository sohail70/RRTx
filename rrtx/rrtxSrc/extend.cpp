#include"rrtx_algorithms.h"
#include"magic_algorithms.h"
#include <iomanip>
Row extend(Matrix& graph, double r, Row& lmc, N& neighbors, obstacle obs, double safetyFactor)
{
	Row new_distance;
	Matrix near_nodes;
	Row dist_to_near_nodes;

	//r = 8; //no need for this. I implemented the shrinkingBall properly
	vector<int> andix; //andix of nodes which are near to the current node
	int i = graph.size()-1;//current node index//size ye done bishtar az andice mishe pas yedoone kam kun
	
	//allocating neighbors set
	neighbors.original_plus.resize(i + 1);
	neighbors.original_minus.resize(i + 1);
	neighbors.running_plus.resize(i + 1);
	neighbors.running_minus.resize(i + 1);
	
	int counter = 0;//for neighbors of current node
	Row current_node = { graph[i][0],graph[i][1] };
	
	////////////////////////////////////////////////////
	//obstacle check
	bool CurrentNodeIsInObs = false;//default value
	if (!obs.circles.empty())
		CurrentNodeIsInObs = obstacleCheck(current_node, obs, safetyFactor);
	////////////////////////////////////////////////////
	bool checkNodeIsInObs=false;//default value
	////////////////////////////////////////////////////

	if (CurrentNodeIsInObs == false)
	{
		for (int j = 0; j < i; j++)//loop through all of the graph's nodes ,right before the last node and find the distance of all nodes to the current node
		{
			Row check_node = { graph[j][0],graph[j][1] };

			////////////////////////////////////////////////////////////////
			//is the check_node(previus nodes) in obs?if it is, newDist->inf
			if (!obs.circles.empty())
				checkNodeIsInObs = obstacleCheck(check_node, obs, safetyFactor);
			////////////////////////////////////////////////////////////////

			float val = euc_dist(check_node, current_node);
			//cout << val << endl; //debug: some times val has some floating point number junc which doesnt goes through the below if loop! //fix: is set it to float to truncate the number a little bit!
			
			////////////////////////////////////////////////////////////////
			if (checkNodeIsInObs == true)
				new_distance.push_back(numeric_limits<double>::infinity());
			else
				new_distance.push_back(val);
			////////////////////////////////////////////////////////////////

			if (val <= r) //say hello to our new neighbors :) //chera kochaktar mosavi: chun min(stepsize, rrt ball) mishe masalan step size va avayel ke man faghat node start ro migiram dardesar mishe! chun daghighan be andaze step size saturate mishe va radius ham daghighan mishe step size!
			{
				andix.push_back(j); //andix used for the next 2 lines
				dist_to_near_nodes.push_back(new_distance[j]);
				near_nodes.push_back(graph[andix[counter]]); //storing near nodes for findParent func

				//lets manage Neighbor sets: what a delight! //of course if you want to use trajectoy generator you need to check for obstacles properly before setting neighbor set.
				neighbors.original_plus[i].push_back(andix[counter]);
				neighbors.original_minus[i].push_back(andix[counter]);
				neighbors.running_plus[andix[counter]].push_back(i);
				neighbors.running_minus[andix[counter]].push_back(i);

				counter += 1;//need this counter for the near_nodes.pushback// counter=0 is the first neighbor 

			}
		}
	}
	//the next loop is for debug--->I optimized the next loop and put it in the top loop for performance
	/*
	vector<vector<double>> near_nodess;

	for (int j = 0; j < andix.size(); j++)
	{
		near_nodess.push_back(graph[andix[j]]);
	}
	*/
		
	
	
	//Find Parent
	//vector<double> current_node_with_specified_parent;//deghat kun graph sotone sevomesh andis ijade node hast vali in sotone sevomesh shomare andis marbot be parent node in radif(radife i) hast!
	graph[i] =findParent(current_node, near_nodes, dist_to_near_nodes, lmc, andix, i);
	
	if (graph[i][3] == (-1) || CurrentNodeIsInObs==true) //no parent found for the current node or the node is obstacle region---->delete this node from the graph and i must not go up 
	{
		//deleting process: delete the last row of the graph matrix
		Matrix::iterator useless_node; //useless nodes row index for deletion process
		useless_node = graph.end() - 1;//-1 is there because end gives us one row past the last row
		graph.erase(useless_node);

		//deletion process: delete the last element of lmc
		lmc.erase(lmc.end() - 1);
		return new_distance;
	}
	//if(fmod(i,1000)==0)//debug
	//cout << i << endl;// debug
	
	return new_distance;
}