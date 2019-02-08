#include "algorithms.hpp"
#include "functions.hpp"
#include <cmath>
#include <math.h>
#define Pi 3.14159265358979323846
double shrinkingBallRadius(int number_of_nodes, double step_size)
{
	double dimension = 2;

	double mu = pow(30, dimension);

	double zeta_d = (pow(Pi, dimension / 2)) / (tgamma((dimension / 2) + 1));
	double factor = 1;
	double gamma_rrt_star = factor * (pow(2 * (1 + 1 / dimension), 1 / dimension)) * pow(mu / zeta_d, 1 / dimension);
	double value = pow(gamma_rrt_star * log10(number_of_nodes) / number_of_nodes, 1 / dimension);

	if (value < step_size && value != 0)
		return value;
	else
		return step_size;
}
