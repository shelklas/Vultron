#pragma once
#include <utility>
#include <vector>
#include "error.h"
#include <tuple>

namespace vultron
{
	class FMS
	{
	private:
		// Vector of route plan.
		// Stored as:
		// Latitude(degrees) [-90..90]
		// Longitude(degrees) [-180..180]
		// Height(m) [no limits] ( Route waypoints MUST be at least 10m (meters) apart. )
		std::vector<std::tuple<double, double, double>> _route;

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
		std::tuple<double, double, double> _loc;

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
		// m/s [-100..100]
		double _velocity = 0;								
		
		// Pitch of aircraft. 
		// Stored as:
		// Degrees [-90..90]
		double _pitch = 0;											   
	public:
		FMS(std::vector<std::tuple<double, double, double>> const & route, std::tuple<double, double, double> const & loc, double const & bearing, double const & height, double const & velocity) : _route(route), _loc(loc), _bearing(bearing), _height(height), _velocity(velocity) {}
		FMS(std::vector < std::tuple <double, double, double>> const & route, std::tuple<double, double, double> const & loc, double const & bearing) : _route(route), _loc(loc), _bearing(bearing) {}
		FMS(std::vector<std::tuple<double, double, double>> const & route) : _route(route) {}
		FMS() {} // For dev testing
		void setRoute(std::vector<std::tuple<double, double, double>> route) { _route = route; }
		std::vector<std::tuple<double, double, double>> getRoute(){return _route;}

		void clearRoute(){_route.clear();}


		void setHeight(double height){std::get<2>(_loc) = height;}
		double getHeight(){return std::get<2>(_loc);}

		void setVelocity(double velocity){_velocity = velocity;}
		double getVelocity(){return _velocity;}

		void setPitch(double pitch){_pitch = pitch;}
		double getPitch(){return _pitch;}

		void setWaypointDistance(double waypointDistance){_waypointDistance = waypointDistance;}
		double getWaypointDistance(){return _waypointDistance;}

		void setLoc(std::tuple<double, double, double> const loc)
		{
			if ((std::get<0>(loc) <= 90 && std::get<0>(loc) >= -90) && (std::get<1>(loc) <= 180 && std::get<1>(loc) >= -180))
				_loc = loc;
			else
				throw error("The set location is beyond the acceptable range.", __FUNCSIG__, __LINE__);
		}
		std::tuple<double, double, double> getLoc(){return _loc;}

		void setBearing(double bearing)
		{
			if (bearing <= 360 && bearing >= 0)
				_bearing = bearing;
			else
				throw error("The set bearing is beyond the acceptable range.", __FUNCSIG__, __LINE__);
		}
		double getBearing(){return _bearing;}

		void nextWaypoint()
		{
			if (static_cast<size_t>(_waypoint) < _route.size())
				_waypoint++;
			else
				throw error("Waypoint beyond route size.", __FUNCSIG__, __LINE__);
		}
		void setWaypoint(int waypoint)
		{
			if (waypoint <= _route.size() && (waypoint >= 0))
				_waypoint = waypoint;
			else
				throw error("Waypoint beyond route size.", __FUNCSIG__, __LINE__);
		}
		int getWaypoint() { return _waypoint; }
	};
	/*
	  @Name: calcDistance(pair<double,double>,pair<double,double>)
	  @Return: Distance in meters [double]
	  @Get: Two decimal gps longitude and latitude pairs [std::pair<double,double>,std::pair<double,double>]
	 */
	double calcDistance(std::tuple<double, double, double > const & loc1, std::tuple<double, double, double> const & loc2);

	/*
	@Name: calcTotalDistance(vector<pair<double,double>>)
	@Return: Total route distance in meters [double]
	@Get: Vector of decimal gps latitude and longitude points
	*/
	double calcTotalDistance(std::vector<std::tuple<double, double, double>> const & path);

	/*
	@Name: calcTripDistance(vector<pair<double,double>>)
	@Return: Individual path lengths for entire route [vector<double>]
	@Get: Vector of decimal gps latitude and longitude points
	*/
	std::vector<double> calcTripDistance(std::vector<std::tuple<double, double, double>> const & path);

	double calcVelocity(std::vector<std::tuple<double, double, double>> const & elapsedPath, double const & elapsedTime);
	double calcBearing(std::tuple<double, double, double> const & loc1, std::tuple<double, double, double> const & loc2);
	std::vector<double> calcTripBearing(std::vector<std::tuple<double, double, double>> const & path);

	void setCurrentBearing(FMS & fms);
	void setCurrentDistance(FMS & fms);

}
