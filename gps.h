#pragma once

#include <tuple>

namespace vultron
{
	using pos_t = std::tuple<double, double, double>;
	class GPS
	{
	private:
		pos_t _loc;
		double _velocity;
	public:
		GPS(pos_t pos);
		GPS();

		pos_t getLoc() { return _loc; }
		double getLongitude() { return std::get<0>(_loc); }
		double getLatitude() { return std::get<1>(_loc); }
		double getHeight() { return std::get<2>(_loc); }
		double getVelocity() { return _velocity; }

		void setLoc();
		void setHeight();
		void setVelocity();
	};
}