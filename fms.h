/*
 * Name: Sheldon Klassen
 * Class: fms.h
 * Description:
 */

#pragma once
#include <utility>
#include <vector>
#include "error.h"
#include <tuple>

namespace vultron
{
	using pos_t = std::tuple<double, double, double>;
	using route_t = std::vector<pos_t>;
	
	double constexpr MIN_DISTANCE_BETWEEN_WAYPOINTS = 10;

	// Helper functions

	/*
	@Name: calcDistance(tuple<double,double,double>,tuple<double,double,double>)
	@Return: Distance in meters [double]
	@Get: Two decimal gps longitude and latitude pairs [std::pair<double,double>,std::pair<double,double>]
	*/
	double calcDistance(pos_t const & loc1, pos_t const & loc2);

	/*
	@Name: calcTotalDistance(vector<tuple<double,double,double>>)
	@Return: Total route distance in meters [double]
	@Get: Vector of decimal gps latitude and longitude points
	*/
	double calcTotalDistance(route_t const & path);

	/*
	@Name: calcTripDistance(vector<tuple<double,double,double>>)
	@Return: Individual path lengths for entire route [vector<double>]
	@Get: Vector of decimal gps latitude and longitude points
	*/
	std::vector<double> calcTripDistance(route_t const & path);

	/*
	@Name: calcVelocity(vector<tuple<double,double,double>>,double)
	@Return: Speed in [m/s]
	@Get: Vector of path aircraft has been in given time frame. Also gets elapsed time passed since last update.
	 */
	double calcVelocity(route_t const & elapsedPath, double const & elapsedTime);

	/*
	@Name: calcBearing(tuple<double,double,double>,tuple<double,double,double>)
	@Return: Bearing in [Degrees]
	@Get: Two decimal gps latitude and longitude points
	*/
	double calcBearing(pos_t const & loc1, pos_t const & loc2);

	/*
	@Name: calcTripBearing(vector<tuple<double,double,double>>,vector<tuple<double,double,double>>)
	@Return: Vector of Bearings in [Degrees]
	@Get: Vector of Bearing for entire route
	*/
	std::vector<double> calcTripBearing(route_t const & path);


	class FMS
	{
	private:
		// Vector of route plan.
		// Stored as:
		// Latitude(degrees) [-90..90]
		// Longitude(degrees) [-180..180]
		// Height(m) [no limits] ( Route waypoints MUST be at least 10m (meters) apart. )
		route_t _route;

		// Waypoint that aircraft is in route to.
		// Stored as:
		// Int
		int _waypoint = 0;

		// Distance from aircraft to next waypoint. With respect to current location.
		// Stored as:
		// Meters [no limits]
		double _waypointDistance = 0;

		// Current location of aicraft in 3D space.
		// Stored as:
		// Latitude(degrees) [-90..90] 
		// Longitude(degrees) [-180..180]
		// Height(m) [no limits]
		pos_t _loc;

		// Current bearing of aircraft. 
		// Stored as:
		// Degrees [0..360]
		double _bearing = 0;

		// Direction of next waypoint. With respect to current location. 
		// Stored as:
		// Degrees [0..360]
		double _waypointBearing = 0;

		// Current speed of aircraft. 
		// Stored as:
		// m/s [no limit (except for speed of light)]
		double _velocity = 0;

		// Pitch of aircraft. 
		// Stored as:
		// Degrees [-90..90]
		double _pitch = 0;

	public:
		FMS(route_t const & route, pos_t const & loc, double const & bearing);
		FMS(route_t const & route);
		FMS() {} // For dev testing

		void setRoute(route_t const & route);
		route_t getRoute() { return _route; }
		void clearRoute() { _route.clear(); }
		void insertNewWaypoint(pos_t waypointLoc, int waypoint);

		void setAltitude(double height) { std::get<2>(_loc) = height; }
		double getAltitude() { return std::get<2>(_loc); }

		void setVelocity(double velocity) { _velocity = velocity; }
		double getVelocity() { return _velocity; }

		void setPitch(double pitch) { _pitch = pitch; }
		double getPitch() { return _pitch; }

		void setLoc(pos_t const loc)
		{
			if ((std::get<0>(loc) <= 90 && std::get<0>(loc) >= -90) && (std::get<1>(loc) <= 180 && std::get<1>(loc) >= -180))
				_loc = loc;
			else
				throw error("Location [" + std::to_string(std::get<0>(loc)) + "] [" + std::to_string(std::get<1>(loc)) + "] is beyond the acceptable range of [-90..90] [-180..180].", __FUNCSIG__, __LINE__);
		}
		pos_t getLoc() { return _loc; }

		void setBearing(double bearing)
		{
			if (bearing <= 360 && bearing >= 0)
				_bearing = bearing;
			else
				throw error("Bearing [" + std::to_string(bearing) + "] is beyond the acceptable range of [0..360].", __FUNCSIG__, __LINE__);
		}
		double getBearing() { return _bearing; }

		void nextWaypoint()
		{
			if (static_cast<size_t>(_waypoint) < _route.size())
				_waypoint++;
			else
				throw error("Waypoint [" + std::to_string(_waypoint + 1) + "] beyond route size of [" + std::to_string(_route.size()) + "].", __FUNCSIG__, __LINE__);
		}
		void setWaypoint(int waypoint)
		{
			if (waypoint <= _route.size() && (waypoint >= 0))
				_waypoint = waypoint;
			else
				throw error("Waypoint [" + std::to_string(waypoint) + "] beyond route size of [" + std::to_string(_route.size()) + "].", __FUNCSIG__, __LINE__);
		}
		int getWaypoint() { return _waypoint; }

		void setWaypointBearing() { _waypointBearing = calcBearing(_loc, _route[_waypoint]); }
		double getWaypointDistance() { return _waypointDistance; }

		void setWaypointDistance() { _waypointDistance = calcDistance(_loc, _route[_waypoint]); }
		double getWaypointBearing() { return _waypointBearing; }
	};


}
