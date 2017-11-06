#pragma once
#include <utility>
#include <vector>
#include "error.h"

namespace vultron
{
	class FMS
	{
	private:
		std::vector<std::pair<double, double>> _route;
		int _waypoint = 0;
		std::pair<double, double> _loc = std::make_pair(0,0); // Latitude/Longitude [-180..180]
		double _bearing = 0; // Degrees [0..360]
		double _height = 0; // Meters [0..1000] (calculated from ground level)
		double _velocity = 0; // m/s [-100..100]
	public:
		FMS(std::vector<std::pair<double, double>> const & route, std::pair<double, double> const & loc, double const & bearing, double const & height, double const & velocity) : _route(route), _loc(loc), _bearing(bearing), _height(height), _velocity(velocity){ }
		FMS(std::vector<std::pair<double, double>> const & route, std::pair<double, double> const & loc, double const & bearing) : _route(route), _loc(loc), _bearing(bearing){ }
		FMS(std::vector<std::pair<double, double>> const & route) : _route(route){ }
		FMS(){ }
		void setRoute(std::vector<std::pair<double,double>> route)
		{
			_route = route;
		}
		std::vector<std::pair<double, double>> getRoute()
		{
			return _route;
		}
		void clearRoute()
		{
			_route.clear();
		}
		void setLoc(std::pair<double,double> loc)
		{
			if((loc.first > 90 || loc.first < -90) &&(loc.second > 180 || loc.second < 180))
				_loc = loc;
			else
				throw error("The set location is beyond the acceptable range.", __FUNCSIG__, __LINE__);
		}
		std::pair<double,double> getLoc()
		{
			return _loc;
		}
		void setBearing(double bearing)
		{
			if (bearing <= 360 && bearing >= 0)
				_bearing = bearing;
			else
				throw error("The set bearing is beyond the acceptable range.", __FUNCSIG__, __LINE__);
		}
		double getBearing()
		{
			return _bearing;
		}
		void setHeight(double height)
		{
			_height = height;
		}
		double getHeight()
		{
			return _height;
		}
		void setVelocity(double velocity)
		{
			_velocity = velocity;
		}
		double getVelocity()
		{
			return _velocity;
		}
		void nextWaypoint()
		{
			if(static_cast<size_t>(_waypoint) < _route.size())
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
	};
	/*
	  @Name: calcDistance(pair<double,double>,pair<double,double>)
	  @Return: Distance in meters [double]
	  @Get: Two decimal gps longitude and latitude pairs [std::pair<double,double>,std::pair<double,double>]
	 */
	double calcDistance(std::pair<double, double> const& loc1, std::pair<double, double> const & loc2);

	/*
	@Name: calcTotalDistance(vector<pair<double,double>>)
	@Return: Total route distance in meters [double]
	@Get: Vector of decimal gps latitude and longitude points
	*/
	double calcTotalDistance(std::vector<std::pair<double, double>> const & path);

	/*
	@Name: calcTripDistance(vector<pair<double,double>>)
	@Return: Individual path lengths for entire route [vector<double>]
	@Get: Vector of decimal gps latitude and longitude points
	*/
	std::vector<double> calcTripDistance(std::vector<std::pair<double, double>> const & path);

	double calcVelocity(std::vector<std::pair<double, double>> const & elapsedPath, double const & elapsedTime);
	double calcBearing(std::pair<double,double> const & loc1,std::pair<double,double> const & loc2);

}
