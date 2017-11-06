#include "utility.h"

namespace utility
{
	double const PI = 3.141592653589793;
	double toRadians(double degrees)
	{
		return (degrees * PI)/180;
	}
	double toDegrees(double radians)
	{
		return (radians * 180) / PI;
	}
	double toKilometer(double meter)
	{
		return meter / 1000;
	}
	double toMeter(double kilometer)
	{
		return kilometer * 1000;
	}
	double toKilometersPerHour(double ms)
	{
		return ms * 3.6;
	}
	double toMetersPerSecond(double kmh)
	{
		return kmh / 3.6;
	}
}