#pragma once
#include <utility>
#include <vector>

namespace vultron
{
	class fms
	{
	private:
		std::pair<double, double> _loc = std::make_pair(0,0);
		double _bearing = 0;
	public:
		void setLoc(std::pair<double,double> loc)
		{
			_loc = loc;
		}
		std::pair<double,double> getLoc()
		{
			return _loc;
		}
		void setBearing(double bearing)
		{
			_bearing = bearing;
		}
		double getBearing()
		{
			return _bearing;
		}
	};
	double calcDistance(std::pair<double, double> loc1, std::pair<double, double> loc2);
	double calcDistance(std::vector<std::pair<double, double>> path);
	double calcBearing(std::pair<double,double> loc1,std::pair<double,double> loc2);
}
