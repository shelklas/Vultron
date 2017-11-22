#pragma once

#include <tuple>
#include <libgpsmm.h>
#include <unistd.h>
#include <math.h>
#include <iostream>

namespace vultron
{
	using pos_t = std::tuple<double, double, double>;
	class GPS
	{
	private:
		pos_t _loc;
    	double _velocity;
        double _track;
        double _time;
        double _climb;
        bool _isOK = false;
       	gpsmm _GPS; 

	public:
		GPS() : _GPS("localhost",DEFAULT_GPSD_PORT) 
		{
			_GPS.send("\$PMTK251,57600*2C\r\n");
			_GPS.send("\$PMTK220,200*2C\r\m");
			system("gpsctl -c 0.2");
            //system("./speed_update");
		}
		pos_t getLoc() { return _loc; }
		double getLongitude() { return std::get<0>(_loc); }
		double getLatitude() { return std::get<1>(_loc); }
		double getHeight() { return std::get<2>(_loc); }
		double getVelocity() { return _velocity; }
        double getTime() { return _time; }

		void getGPSData();
		void update();
	};
}
