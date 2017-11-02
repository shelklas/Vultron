#include "utility.h"
#include <math.h>

namespace utility
{
	double toRadians(double degrees)
	{
		return (degrees * 3.141592653589793)/180;
	}
	double toDegrees(double radians)
	{
		return (radians * 180) / 3.141592653589793;
	}
}