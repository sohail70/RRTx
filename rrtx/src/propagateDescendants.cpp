#include "algorithms.hpp"
#include "functions.hpp"
#include <algorithm>
using namespace std;

void propagateDescendants(Matrix &graph, Matrix &Q, N &neighbors, vector<int> &orphansIndex, Row &gValue, Row &lmc)
{
	//add the effected child of the messed up nodes to the orphanage!--->ye seri nodeha ke ghashang ro
	//obs hastan, ina hala ye seri bache daran (momkene bachashon hamoon khodeshon bashan ke ye bar hesab mishan),in bacheha
	//dar vaghe hamoon E0 hast ye joraee! ke ma be orphange ezafe mikunim
	//bayad deghat kuni ke for loop aval ye loope afzayeshi ast va vaghti ye laye child haro be orphanage ezef kari
	//bayad layehaye badi yani child haye in node haye orphan ro ham be orphanage ezaf kuni
	//ta zamani ke dg child ee namoone!--->algorithm matlabam khube va rosh fekr shode-->hamoono piade mikunam!

	//ROS_WARN("flag 1111111");
	//so let's fill up the orphange with the childs of the messed up nodes --->and again with their childs too
	vector<int> temp(orphansIndex.begin(), orphansIndex.end()); //for conversion from vector<int> to double I used this technique because
	// temp=currentChild woudln't work in that case!
	vector<int> vec;
	vector<int> currentChilds;
	vector<int> wholeChilds;
	int j = 1;

	while (!temp.empty()) //my version of traversing the tree--> i don't know maybe I wrote BFS unconciously :)
	{
		vec.clear();
		vec = findRow(graph, temp[j - 1]); //nokte kheili mohem-->bug: faghat to sotone 4 om bayad begardi-->sotone 1 va 2 ham momkene bug ijad kune chun x=47 mokene barabar ba node 47 ham bashe!
		//delete current node from vec-->because this is duplication!
		int whichOne = findInVec(vec, temp[j - 1]);
		vector<int>::iterator deletingNode;
		deletingNode = vec.begin() + whichOne;
		vec.erase(deletingNode);
		//also delete the nodes in vec which are the same as temp nodes!//if you don't use this precedure you are gonna end up in INFINITY loop :-/
		for (int i = 0; i < temp.size(); i++)
		{
			int whichOne = findInVec(vec, temp[i]);
			if (whichOne != -1)
			{
				vector<int>::iterator deletingNode;
				deletingNode = vec.begin() + whichOne;
				vec.erase(deletingNode);
			}
		}

		//bug
		for (int i = 0; i < vec.size(); i++)
			if (vec[i] == 0)
				cout << "why?";

		//if(!vec.empty())
		currentChilds.insert(currentChilds.end(), vec.begin(), vec.end());

		/*  //another approach-->chun element aval current child har azgahi sefr mishod-->ke unam bekhatere sotone 1 va 2 graph bod dar func e findRow!
		for (int w = 0; w < vec.size(); w++)
			currentChilds.push_back(vec[w]);
			*/
		//unique(currentChilds.begin(), currentChilds.end()); //no need fro this-->maybe use it later if you want
		j++;

		//buuuugggg-->currentChild keeps adding zero to the begining!
		for (int i = 0; i < currentChilds.size(); i++)
			if (currentChilds[i] == 0)
				cout << "why?";

		if (j > temp.size())
		{
			/*
			for (int i = 0; i < wholeChilds.size(); i++)
				for (int k = 0; k < currentChilds.size(); k++)
					if (wholeChilds[i] == currentChilds[k])
						cout << "why?";
			*/
			temp = currentChilds;
			wholeChilds.insert(wholeChilds.end(), temp.begin(), temp.end());
			//sort(wholeChilds.begin(), wholeChilds.end());
			currentChilds.clear();
			currentChilds.resize(0);
			j = 1;
		}
		/*
		if (temp.empty()) //no more children have found, so let's get the hell outta here!
			break;
			*/
	}
	//ROS_WARN("flag 222222222");
	//add the wholeChild(child of the messed up nodes) to the orphanage
	orphansIndex.insert(orphansIndex.end(), wholeChilds.begin(), wholeChilds.end());

	//second loop

	vector<int> wholePlusNeighborsAndParent; //whole minus neighbors of someIndexNode without the parent node of someIndexNode

	//for (int o=0;o<orphansIndex.size();o++)
	//	ROS_WARN("flag 3333333333333333 %i",orphansIndex[o]);

	for (int i = 0; i < orphansIndex.size(); i++)
	{
		//ROS_WARN("flag 3333333333333333 cur orphan %i", orphansIndex[i]);
		//merging the running and original plus neighbors
		wholePlusNeighborsAndParent = neighbors.original_plus.at(orphansIndex[i]);
		for (int k = 0; k < neighbors.running_plus.at(orphansIndex[i]).size(); k++) //adding the running minus to the original ones to have a MINUS PACKAGE
		{
			wholePlusNeighborsAndParent.push_back(neighbors.running_plus.at(orphansIndex[i]).at(k));
			//ROS_WARN("flag 33333333333 neighbor %i", neighbors.running_plus.at(orphansIndex[i]).at(k));
		}
		//add the parent of v to the wholePlusNeighbors
		ROS_WARN("graph %f %f %f %f", graph[orphansIndex[i]][0], graph[orphansIndex[i]][1], graph[orphansIndex[i]][2], graph[orphansIndex[i]][3]);
		if (graph[orphansIndex[i]][3] != numeric_limits<double>::infinity())
			wholePlusNeighborsAndParent.push_back(graph[orphansIndex[i]][3]); //parent of v added
		//for (int o = 0; o < wholePlusNeighborsAndParent.size(); o++)
			//ROS_WARN("flag 44444444444444444444 %i", wholePlusNeighborsAndParent[o]);
		//loop through the neighbors of this orphan node to inf the gValues and add them to the queue so that they can find new parents!
		for (int j = 0; j < wholePlusNeighborsAndParent.size(); j++)
		{

			int u = wholePlusNeighborsAndParent[j];
			//ROS_WARN("flag 555555555555555555555 %i", u);
			gValue[u] = numeric_limits<double>::infinity();
			//ROS_WARN("flag 666666666666666666666666");
			verrifyQueue(Q, gValue[u], lmc[u], u);
			//ROS_WARN("flag 777777777777777777777777");
		}

		//third loop!--->dealing with the current orphan node it self---> inf (both) the gValue and lmc to inf--->why both?
		//because orphans are messed up and nothing can be done for them
		gValue[orphansIndex[i]] = numeric_limits<double>::infinity();
		lmc[orphansIndex[i]] = numeric_limits<double>::infinity();

		//I didn't consider building any child nodes vector or sth!--->but maybe I should add child index as the 5th column of graph in the extend function!
		//(if i do it, i should delete the child in the next line, so be on guard!)--> nothing important though!
		graph[orphansIndex[i]][3] = numeric_limits<double>::infinity(); //orphan nodes parent is now inf!;
	}

	//third loop---->this can be done in the above loop

	//ROS_WARN("flag 666666666");
}
