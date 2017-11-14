#include "gps.h"

namespace vultron
{
	GPS::GPS()
	{

	}
	GPS::GPS(pos_t loc)
	{
		_loc = loc;
	}
	void GPS::setLoc()
	{
	}
	void GPS::setVelocity()
	{
	}
	void GPS::update()
	{
		this->setLoc();
		this->setVelocity();
	}
}