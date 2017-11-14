/*
* Name: Sheldon Klassen
* Class: fms.cpp
* Description:
*/


#include "fms.h"
#include "utility.h"
#include <math.h>


namespace vultron
{
	FMS::FMS(route_t const & route, pos_t const & loc, double const & bearing)
	{
		this->setRoute(route);
		this->setHeading(bearing);
		this->setLoc(loc);
	}
	FMS::FMS(route_t const & route)
	{
		this->setRoute(route);
	}
	void FMS::setRoute(route_t const & route)
	{
		// User may give a blank route
		if (route.size() == 0) { return; }
		// Check if waypoints are less than set minimum distance apart
		for (route_t::const_iterator it = route.begin(); it != route.end() - 1; ++it)
			if (calcDistance(*it, *(it + 1)) < MIN_DISTANCE_BETWEEN_WAYPOINTS)
				throw error("Waypoint distance for waypoints [" + std::to_string(std::distance(route.begin(), it)) + "] and [" + std::to_string(std::distance(route.begin(), it + 1)) + "] exceed minimum distance between waypoints of [" + std::to_string(MIN_DISTANCE_BETWEEN_WAYPOINTS) + "m].", __FUNCSIG__, __LINE__);

		// Check if waypoints are outside range of longitude and latitude coordinates
		for (route_t::const_iterator it = route.begin(); it != route.end(); ++it)
			if (!((std::get<0>(*it) <= 90 && std::get<0>(*it) >= -90) && (std::get<1>(*it) <= 180 && std::get<1>(*it) >= -180)))
				throw error("Waypoint [" + std::to_string(std::distance(route.begin(), it)) + "] with values [" + std::to_string(std::get<0>(*it)) + "] [" + std::to_string(std::get<1>(*it)) + "] is beyond the acceptable range of [-90..90] [-180..180].", __FUNCSIG__, __LINE__);
		_route = route;
	}
	void FMS::insertWaypoint(pos_t waypointLoc, int waypoint)
	{
		route_t route = this->getRoute();
		if (waypoint > route.size() || (waypoint < 0))
			throw error("Waypoint [" + std::to_string(waypoint) + "] is beyond route size of [" + std::to_string(_route.size()) + "].", __FUNCSIG__, __LINE__);
		if (waypoint == 0)
		{
			if (calcDistance(waypointLoc, route[0]) < MIN_DISTANCE_BETWEEN_WAYPOINTS)
				throw error("Waypoint distance for insert [" + std::to_string(waypoint) + "] and [1] exceed minimum distance between waypoints of [" + std::to_string(MIN_DISTANCE_BETWEEN_WAYPOINTS) + "m].", __FUNCSIG__, __LINE__);
		}
		else if (waypoint == route.size())
		{
			if (calcDistance(route[route.size() - 1], waypointLoc) < MIN_DISTANCE_BETWEEN_WAYPOINTS)
				throw error("Waypoint distance for insert [" + std::to_string(waypoint) + "] and [" + std::to_string(route.size()) + "] exceed minimum distance between waypoints of [" + std::to_string(MIN_DISTANCE_BETWEEN_WAYPOINTS) + "m].", __FUNCSIG__, __LINE__);
		}
		else
		{
			for (size_t i = waypoint - 1; i < waypoint + 1; ++i)
				if (calcDistance(route[i], waypointLoc) < MIN_DISTANCE_BETWEEN_WAYPOINTS)
					throw error("Waypoint distance for insert [" + std::to_string(waypoint) + "] and [" + std::to_string(i) + "] exceed minimum distance between waypoints of [" + std::to_string(MIN_DISTANCE_BETWEEN_WAYPOINTS) + "m].", __FUNCSIG__, __LINE__);
		}
		_route.insert((_route.begin() + waypoint), waypointLoc);

	}
	void FMS::removeWaypoint(int waypoint)
	{
		route_t route = this->getRoute();
		if (waypoint >= route.size() || waypoint < 0)
			throw error("Waypoint [" + std::to_string(waypoint) + "] is beyond route size of [" + std::to_string(_route.size()) + "].", __FUNCSIG__, __LINE__);
		if (waypoint < route.size() - 1 && waypoint >= 1) // Do not need to check fringes[0..vector.size()] of vector
			if (calcDistance(route[waypoint - 1], route[waypoint + 1]) < MIN_DISTANCE_BETWEEN_WAYPOINTS)
				throw error("Waypoint distance for insert [" + std::to_string(waypoint - 1) + "] and [" + std::to_string(waypoint + 1) + "] exceed minimum distance between waypoints of [" + std::to_string(MIN_DISTANCE_BETWEEN_WAYPOINTS) + "m].", __FUNCSIG__, __LINE__);
		_route.erase(_route.begin() + waypoint);
	}
	void FMS::setRateOfTurn()
	{
		_rateOfTurn = (1091 * tan(utility::toRadians(std::get<2>(_axis)))) / utility::toKnots(_velocity);
	}	
	void FMS::update()
	{
		/*
		Need GPS and Sensor class data in order to update FMS.
		 */
		gps.update();
		sensor.update();
		if (this->getWaypointDistance() < MIN_DISTANCE_TO_WAYPOINT)
			this->nextWaypoint();
		this->setWaypointDistance();
		this->setWaypointHeading();
		this->setAxis(sensor.getAxis());
		this->setLoc(gps.getLoc());
		this->setVelocity(gps.getVelocity());
		this->setRateOfTurn();
	}
	double calcDistance(pos_t const & loc1, pos_t const &  loc2)
	{
		// Calculate distance using Haversine formula
		double const R = 6371000; // Radius of earth
		double const phi1 = utility::toRadians(std::get<0>(loc1));
		double const phi2 = utility::toRadians(std::get<0>(loc2));
		double const deltaPhi = utility::toRadians(std::get<0>(loc2) - std::get<0>(loc1));
		double const deltaLambda = utility::toRadians(std::get<1>(loc2) - std::get<1>(loc1));
		double const a = sin(deltaPhi / 2) * sin(deltaPhi / 2) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);
		double const c = 2 * atan2(sqrt(a), sqrt(1 - a));
		double d = R * c;

		// Adjust distance for change in altitude using Pythagoras' Theorem 
		double const deltaAltitude = std::abs(std::get<2>(loc2) - std::get<2>(loc1));
		d = std::sqrt(std::pow(deltaAltitude, 2) + std::pow(d, 2));
		return d;
	}
	double calcTotalDistance(route_t const & path)
	{
		double totalPathLength = 0;
		if (path.size() == 0 || path.size() == 1)
			return totalPathLength; // Return 0 length. As no coordinates and 1 coordinate will not give a length
		for (route_t::const_iterator it = path.begin(); it != path.end() - 1; ++it)
			totalPathLength += calcDistance(*it, *(it + 1));
		return totalPathLength;
	}
	std::vector<double> calcTripDistance(route_t const & path)
	{
		std::vector<double> trip;
		for (route_t::const_iterator it = path.begin(); it != path.end() - 1; ++it)
			trip.push_back(calcDistance(*it, *(it + 1)));
		return trip;
	}
	double calcVelocity(route_t const & elapsedPath, double const & elapsedTime)
	{
		double elapsedDistanceCovered = calcTotalDistance(elapsedPath);
		double velocity = elapsedDistanceCovered / elapsedTime;
		return velocity;
	}
	double calcBearing(pos_t const & loc1, pos_t const & loc2)
	{
		double const phi1 = utility::toRadians(std::get<0>(loc1));
		double const phi2 = utility::toRadians(std::get<0>(loc2));
		double const deltaLambda = utility::toRadians(std::get<1>(loc2) - std::get<1>(loc1));

		double const y = sin(deltaLambda) * cos(phi2);
		double const x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
		double const bearing = utility::toDegrees(atan2(y, x));
		return utility::mod((bearing + 360), 360); // Normalize from [-180..180] to [0..360]
	}
	std::vector<double> calcTripBearing(route_t const & path)
	{
		std::vector<double> tripBearing;
		for (route_t::const_iterator it = path.begin(); it != path.end() - 1; ++it)
			tripBearing.push_back(calcBearing(*it, *(it + 1)));
		return tripBearing;
	}
	double calcStallSpeed(double airDensity, double wingArea, double clMax, double weight, double gravity)
	{
		double vstall = sqrt((2 * weight * gravity) / (airDensity * wingArea * clMax));
		return vstall;
	}
	double calcDeltaDirection(double heading, double waypointHeading)
	{
		double clockwise = heading - waypointHeading;
		double counterClockwise = heading >= waypointHeading ? (heading-360) - waypointHeading: heading - (waypointHeading -360);
		return abs(clockwise) <= abs(counterClockwise) ? clockwise : counterClockwise; // Test what direction is closer
	}

}
