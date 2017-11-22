#include "utility.h"
#include "fms.h"
#include "error.h"
#include <tuple>
#include <vector>
#include <unistd.h>
#include <stdexcept>
	int main()
	{
	    std::vector < std::tuple < double, double, double >> path;
        
        // Insert route here

        path.push_back(std::make_tuple(43.37999, -80.94842, 10));
        path.push_back(std::make_tuple(43.38019, -80.96310, 10));
        path.push_back(std::make_tuple(43.38933, -80.96314, 10));
        path.push_back(std::make_tuple(43.38939, -80.96744, 10));
        path.push_back(std::make_tuple(43.40792, -80.95276, 10));
        path.push_back(std::make_tuple(43.38104, -80.95920, 10));
	    
        // End of route

       	vultron::FMS fms;
		fms.setRoute(path);
		while(fms.getWaypoint() !=path.size())
        {
         try
         {
            usleep(200000);
            fms.update();
            std::cout << std::endl;
            std::tuple<double,double,double> loc = fms.getLoc();
            std::tuple<double,double,double> axis = fms.getAxis();
            std::cout << "TME: " << std::to_string(fms.getTime()) << std::endl;
            std::cout << "LAT: " << std::to_string(std::get<0>(loc)) << std::endl;
            std::cout << "LON: " << std::to_string(std::get<1>(loc)) << std::endl;
            std::cout << "HGT: " << std::to_string(std::get<2>(loc)) << std::endl;
            std::cout << "SPD: " << std::to_string(fms.getVelocity()) << std::endl;
            std::cout << "YAW: " << std::to_string(std::get<0>(axis)) << std::endl;
            std::cout << "PCH: " << std::to_string(std::get<1>(axis)) << std::endl;
            std::cout << "ROL: " << std::to_string(std::get<2>(axis)) << std::endl;
            std::cout << "NEXT WAYPOINT: " << fms.getWaypoint() << std::endl;
            std::cout << "DIST: " << fms.getWaypointDistance() << " HEAD: " << fms.getWaypointHeading() << std::endl;
            std::cout << std::endl;
         }
           catch(vultron::error e)
           {
              std::cout << e.toString() << std::endl; 
            }
        }
	return 0;
	}

