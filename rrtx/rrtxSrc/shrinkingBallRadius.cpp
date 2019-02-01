#include"rrtx_algorithms.h"
#include"magic_algorithms.h"
#include<cmath>
#include<math.h>
# define Pi  3.14159265358979323846
double shrinkingBallRadius(int number_of_nodes,double step_size)
{
	double dimension = 2; //state space is x & y--> I guess :)----> I'm not sure about this
	
	double mu = pow(100,dimension); //this must be evaluated for every dimension---> (maxBound of dim1-min bound of dim1)*(maxBound of dim2-min bound of dim2)*(maxbound of dim3-minBound of dim3)*....
	//manzoor az maxBound va minbound hamoon lower bound va upper bound vase sample hast----yani hamoon range ee ke vase x sample mikuni ke 100 hast taghriban, vase y ham hamine ke 100 hast...pas mishe 100 ^2;
	//dar vaghe mu baste be environemnt dare va age ziad beshe, r ham ziad mishe ta mohit bishtari poshesh dade beshe ke manteghi ham hast
	double zeta_d =( pow(Pi, dimension/2) )/ (tgamma((dimension / 2) + 1));
	double factor = 200; //I guess it must be more than one to satisfy the rrt* criteria----> in factor bala age bashe vazeh hast ke r ro ziad mikune va hamgaraee bala mire vali khub dardesar darim moghe barkhurd ba mavane ke bayad onja ham robust bashim!
	double gamma_rrt_star = factor*(pow(2 * (1 + 1 / dimension), 1 / dimension))*pow(mu / zeta_d, 1 / dimension);
	double value = pow(gamma_rrt_star*log10(number_of_nodes) / number_of_nodes, 1 / dimension);
	
	if (value < step_size && value!=0)
	{
		//cout << value << endl; //debug
		return value;
	}
	else
	{
		//cout << step_size << endl; //debug
		return step_size;
	}
}

