/*
* Name: Sheldon Klassen
* Class: utility.h
* Description:
*/

#pragma once
#include "fms.h"
#include <vector>

namespace utility
{
	double toRadians(double degrees);
	double toDegrees(double radians);
	double toMeter(double kilometer);
	double toKilometer(double meter);
	double toKnots(double metersPerSecond);
	double mod(double x, double y);
	void printRoutePositions(vultron::route_t & route);
}
