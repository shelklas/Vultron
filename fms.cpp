#include "fms.h"
#include "utility.h"
#include <math.h>


namespace vultron
{
	FMS::FMS(std::vector<std::tuple<double, double, double>> const & route, std::tuple<double, double, double> const & loc, double const & bearing)
	{
		this->setRoute(route);
		this->setBearing(bearing);
		this->setLoc(loc);
	}
	FMS::FMS(std::vector<std::tuple<double, double, double>> const & route)
	{
		this->setRoute(route);
	}
	void FMS::setRoute(std::vector<std::tuple<double, double, double>> const & route)
	{
		// User may give a blank route
		if (route.size() == 0) { return; }
		// Check if waypoints are less than set minimum distance apart
		for (std::vector<std::tuple<double, double, double>>::const_iterator it = route.begin(); it != route.end() - 1; ++it)
			if (calcDistance(*it, *(it + 1)) < MIN_DISTANCE_BETWEEN_WAYPOINTS)
				throw error("Waypoint distance for waypoints [" + std::to_string(std::distance(route.begin(), it)) + "] and [" + std::to_string(std::distance(route.begin(), it + 1)) + "] exceed minimum distance between waypoints of [" + std::to_string(MIN_DISTANCE_BETWEEN_WAYPOINTS) + "m].", __FUNCSIG__, __LINE__);

		// Check if waypoints are outside range of longitude and latitude coordinates
		for (std::vector<std::tuple<double, double, double>>::const_iterator it = route.begin(); it != route.end(); ++it)
			if (!((std::get<0>(*it) <= 90 && std::get<0>(*it) >= -90) && (std::get<1>(*it) <= 180 && std::get<1>(*it) >= -180)))
				throw error("Waypoint [" + std::to_string(std::distance(route.begin(), it)) + "] with values [" + std::to_string(std::get<0>(*it)) + "] [" + std::to_string(std::get<1>(*it)) + "] is beyond the acceptable range.", __FUNCSIG__, __LINE__);
		_route = route;
	}
	double calcDistance(std::tuple<double, double, double> const & loc1, std::tuple<double, double, double> const &  loc2)
	{
		double const R = 6371000; // Radius of earth
		double const phi1 = utility::toRadians(std::get<0>(loc1));
		double const phi2 = utility::toRadians(std::get<0>(loc2));
		double const deltaPhi = utility::toRadians(std::get<0>(loc2) - std::get<0>(loc1));
		double const deltaLambda = utility::toRadians(std::get<1>(loc2) - std::get<1>(loc1));

		double const a = sin(deltaPhi / 2) * sin(deltaPhi / 2) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);
		double const c = 2 * atan2(sqrt(a), sqrt(1 - a));
		double const d = R * c;
		return d;
	}
	double calcTotalDistance(std::vector<std::tuple<double, double, double>> const & path)
	{
		double totalPathLength = 0;
		if (path.size() == 0 || path.size() == 1)
			return totalPathLength; // Return 0 length. As no coordinates and 1 coordinate will not give a length
		for (std::vector<std::tuple<double, double, double>>::const_iterator it = path.begin(); it != path.end() - 1; ++it)
			totalPathLength += calcDistance(*it, *(it + 1));
		return totalPathLength;
	}
	std::vector<double> calcTripDistance(std::vector<std::tuple<double, double, double>> const & path)
	{
		std::vector<double> trip;
		for (std::vector<std::tuple<double, double, double>>::const_iterator it = path.begin(); it != path.end() - 1; ++it)
			trip.push_back(calcDistance(*it, *(it + 1)));
		return trip;
	}
	double calcVelocity(std::vector<std::tuple<double, double, double>> const & elapsedPath, double const & elapsedTime)
	{
		double elapsedDistanceCovered = calcTotalDistance(elapsedPath);
		double velocity = elapsedDistanceCovered / elapsedTime;
		return velocity;
	}
	double calcBearing(std::tuple<double, double, double> const & loc1, std::tuple<double, double, double> const & loc2)
	{
		double const phi1 = utility::toRadians(std::get<0>(loc1));
		double const phi2 = utility::toRadians(std::get<0>(loc2));
		double const deltaLambda = utility::toRadians(std::get<1>(loc2) - std::get<1>(loc1));

		double const y = sin(deltaLambda) * cos(phi2);
		double const x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
		double const bearing = utility::toDegrees(atan2(y, x));
		return utility::mod((bearing + 360), 360); // Normalize from [-180..180] to [0..360]
	}
	std::vector<double> calcTripBearing(std::vector<std::tuple<double, double, double>> const & path)
	{
		std::vector<double> tripBearing;
		for (std::vector<std::tuple<double, double, double>>::const_iterator it = path.begin(); it != path.end() - 1; ++it)
			tripBearing.push_back(calcBearing(*it, *(it + 1)));
		return tripBearing;
	}
}
