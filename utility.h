/*
* Name: Sheldon Klassen
* Class: utility.h
* Description:
*/

#pragma once
#include <tuple>
#include <vector>

namespace utility
{
	double toRadians(double degrees);
	double toDegrees(double radians);
	double toMeter(double kilometer);
	double toKilometer(double meter);
	double mod(double x, double y);
	void printRoutePositions(std::vector<std::tuple<double, double, double>> & route);
}
