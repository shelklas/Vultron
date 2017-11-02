#include "calc.h"
#include "utility.h"
#include <algorithm>
#include <math.h>


namespace vultron
{
	double calcDistance(std::pair<double, double> loc1, std::pair<double, double> loc2)
	{
		double R = 6378137;
		double phi1 = utility::toRadians(loc1.first);
		double phi2 = utility::toRadians(loc2.first);
		double deltaPhi = utility::toRadians(loc2.first - loc1.first);
		double deltaLambda = utility::toRadians(loc2.second - loc1.second);
		
		double a = sin(deltaPhi / 2) * sin(deltaPhi / 2) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);
		double c = 2 * atan2(sqrt(a), sqrt(1 - a));
		double d = R * c;
		return d;
	}
	double calcDistance(std::vector<std::pair<double, double>> path)
	{
		double totalPathLength = 0;
		if (path.size() == 0 || path.size() == 1)
			return 0;
		for (std::vector<std::pair<double, double>>::iterator it = path.begin(); it != path.end() - 1; ++it)
			totalPathLength += calcDistance(*it, *(it+1));
		return totalPathLength;
	}
}
