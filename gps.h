#pragma once

#include <tuple>

namespace vultron
{
	using pos_t = std::tuple<double, double, double>;
	class GPS
	{
	private:
		pos_t _pos;
		double _velocity;
	public:
		GPS(pos_t pos);
		GPS();

		pos_t getPos() { return _pos; }
		double getLongitude() { return std::get<0>(_pos); }
		double getLatitude() { return std::get<1>(_pos); }
		double getHeight() { return std::get<2>(_pos); }
		double getVelocity() { return _velocity; }

		void setPos();
		void setHeight();
		void setVelocity();
	};
}