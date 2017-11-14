#pragma once
#include <tuple>

namespace vultron
{
	using axis_t = std::tuple<double, double, double>;
	class Sensor
	{
	private:
		axis_t _axis;

		// double _speed; // Possibly will be using pitol tube for speed governing
	public:
		Sensor();

		double getHeading() { return std::get<0>(_axis); }
		double getPitch() { return std::get<1>(_axis); };
		double getRoll() { return std::get<2>(_axis); };
		axis_t getAxis() { return _axis; }

		void setAxis();
		void setHeading();
		void setPitch();
		void setRoll();
		void update();
	};
}