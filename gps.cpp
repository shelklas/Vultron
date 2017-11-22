#include "error.h"
#include "gps.h"
#include <tuple>
#include <libgpsmm.h>
#include <unistd.h>
namespace vultron
{
    void GPS::getGPSData()
	{
        if(_GPS.stream(WATCH_ENABLE | WATCH_JSON) == NULL)
           system("./gps_init");
            // throw error("NO GPSD RUNNING",__FUNCTION__,__LINE__);
        struct gps_data_t * readInData;
        if((readInData = _GPS.read()) == NULL)
            throw error("GPS read error",__FUNCTION__,__LINE__);
        else
            while((readInData = _GPS.read()) == NULL || (readInData->fix.mode < 1))
                    {}
        /*if(isnan(readInData->fix.latitude))
        {
            _isOK = false;
            std::cout <<"GPS data is stale... Getting location..." << std::endl;
            while(readInData->fix.mode == STATUS_NO_FIX)
            {
                std::cout << "ALIGNING..." << std::endl;
                if(!_GPS.waiting(200000)) continue;
                readInData = _GPS.read();
            }
            _isOK = true;
        }*/
        
        _loc = std::make_tuple(readInData->fix.latitude,readInData->fix.longitude,readInData->fix.altitude);
        _velocity = readInData->fix.speed;
        _track = readInData->fix.track;
        _time = readInData->fix.time;
        _climb = readInData->fix.climb;

        //std::cout << _GPS.data() << std::endl;
	}
	void GPS::update()
	{
		this->getGPSData();
	}
}

