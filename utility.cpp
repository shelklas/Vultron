/*
* Name: Sheldon Klassen
* Class: utility.cpp
* Description:
*/

#include "utility.h"
#include <cmath>
#include <vector>
#include <ostream>
#include <string>
#include <iostream>

namespace utility
{
	double const PI = 3.141592653589793;
	double toRadians(double degrees) { return (degrees * PI) / 180; }
	double toDegrees(double radians) { return (radians * 180) / PI; }
	double toKilometer(double meter) { return meter / 1000; }
	double toMeter(double kilometer) { return kilometer * 1000; }
	double toKilometersPerHour(double ms) { return ms * 3.6; }
	double toMetersPerSecond(double kmh) { return kmh / 3.6; }
	double toKnots(double metersPerSecond) { return metersPerSecond *1.94384449; }
	double mod(double a, double b) { return a - b * floor(a / b); }
	void printRoutePositions(vultron::route_t& route)
	{
		for (vultron::route_t::iterator it = route.begin(); it != route.end(); ++it)
			std::cout << std::to_string(std::get<0>(*it)) + ", " + std::to_string(std::get<1>(*it)) << std::endl;
	}
}
