#include "fms.h"
#include "utility.h"
#include <math.h>


namespace vultron
{
	double calcDistance(std::tuple<double, double,double> const & loc1, std::tuple<double, double,double> const &  loc2)
	{
		double R = 6371000;
		double phi1 = utility::toRadians(std::get<0>(loc1));
		double phi2 = utility::toRadians(std::get<0>(loc2));
		double deltaPhi = utility::toRadians(std::get<0>(loc2) - std::get<0>(loc1));
		double deltaLambda = utility::toRadians(std::get<1>(loc2) - std::get<1>(loc1));
		
		double a = sin(deltaPhi / 2) * sin(deltaPhi / 2) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);
		double c = 2 * atan2(sqrt(a), sqrt(1 - a));
		double d = R * c;
		return d;
	}
	double calcTotalDistance(std::vector<std::tuple<double, double,double>> const & path)
	{
		double totalPathLength = 0;
		if (path.size() == 0 || path.size() == 1)
			return 0;
		for (std::vector<std::tuple<double, double,double>>::const_iterator it = path.begin(); it != path.end() - 1; ++it)
			totalPathLength += calcDistance(*it, *(it+1));
		return totalPathLength;
	}
	std::vector<double> calcTripDistance(std::vector<std::tuple<double, double,double>> const & path)
	{
		std::vector<double> trip;
		for (std::vector<std::tuple<double,double,double>>::const_iterator it = path.begin(); it != path.end()-1; ++it)
			trip.push_back(calcDistance(*it, *(it + 1)));
		return trip;
	}
	double calcVelocity(std::vector<std::tuple<double, double,double>> const & elapsedPath, double const & elapsedTime)
	{
		double elapsedDistanceCovered = calcTotalDistance(elapsedPath);
		double velocity = elapsedDistanceCovered / elapsedTime;
		return velocity;
	}
	double calcBearing(std::tuple<double, double,double> const & loc1, std::tuple<double, double,double> const & loc2)
	{
		double phi1 = utility::toRadians(std::get<0>(loc1));
		double phi2 = utility::toRadians(std::get<0>(loc2));
		double deltaLambda = utility::toRadians(std::get<1>(loc2) - std::get<1>(loc1));

		double y = sin(deltaLambda) * cos(phi2);
		double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
		double bearing = utility::toDegrees(atan2(y, x));
		return utility::mod((bearing + 360), 360); // Normalize from [-180..180] to [0..360]
	}
	std::vector<double> calcTripBearing(std::vector<std::tuple<double,double,double>> const & path)
	{
		std::vector<double> tripBearing;
		for (std::vector<std::tuple<double, double,double>>::const_iterator it = path.begin(); it != path.end() - 1; ++it)
			tripBearing.push_back(calcBearing(*it, *(it + 1)));
		return tripBearing;
	}
	void setCurrentBearing(FMS & fms)
	{
		fms.setBearing(calcBearing(fms.getLoc(), fms.getRoute()[fms.getWaypoint()]));
	}
	void setCurrentDistance(FMS& fms)
	{
		fms.setWaypointDistance(calcDistance(fms.getLoc(), fms.getRoute()[fms.getWaypoint()]));
	}
}
