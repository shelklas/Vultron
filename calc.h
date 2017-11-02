#pragma once
#include <utility>
#include <vector>

namespace vultron
{
	double calcDistance(std::pair<double,double> loc1, std::pair<double, double> loc2);
	double calcDistance(std::vector<std::pair<double, double>> path);
}
