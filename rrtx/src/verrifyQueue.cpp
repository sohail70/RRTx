#include "algorithms.hpp"
#include "functions.hpp"
#include <algorithm>

void verrifyQueue(Matrix &Q , double gValue , double lmc , double node)
{
	Row row_col;
	row_col = find(Q, node);
	double row_index = row_col[0];
	double col_index = row_col[1];

	if (row_index != -1 && col_index != -1)
	{
		double temp = min(gValue,lmc);
		Q[row_index][0] = temp;
		Q[row_index][1] = gValue;
		Q[row_index][2] = node;
	}
	else
	{
		Row temp = { min(gValue,lmc),gValue,node};
		Q.push_back(temp);
	}
	sortrows(Q, 0);
}
