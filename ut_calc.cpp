#define BOOST_TEST_MODULE VultronUnitTest
#include "fms.h"
#include "utility.h"
#include <boost/test/auto_unit_test.hpp>
#include <vector>


using namespace vultron;
BOOST_AUTO_TEST_CASE(single_distance_test)
{
	// Test small distance
	BOOST_CHECK_CLOSE(820.5, vultron::calcDistance(std::make_tuple(43.38016054259229, -80.96304200426027, 10), std::make_tuple(43.379957798085755, -80.95290325418398, 10)), 0.1);
	// Test large distance
	BOOST_CHECK_CLOSE(10300000, vultron::calcDistance(std::make_tuple(43.38100270482399, -80.95941565767214, 10), std::make_tuple(35.71073183384689, 139.7312071352967, 10)), 0.1);

}
BOOST_AUTO_TEST_CASE(multi_distance_many_point_small_test)
{
	FMS fms;
	std::vector < std::tuple < double, double, double >> path;
	path.push_back(std::make_tuple(43.37999, -80.94842, 10)); // Starting Point
	path.push_back(std::make_tuple(43.38019, -80.96310, 10));
	path.push_back(std::make_tuple(43.38933, -80.96314, 10));
	path.push_back(std::make_tuple(43.38939, -80.96744, 10));
	path.push_back(std::make_tuple(43.40792, -80.95276, 10));
	path.push_back(std::make_tuple(43.38104, -80.95920, 10));
	fms.setRoute(path);
	BOOST_CHECK_CLOSE(7'966, calcTotalDistance(fms.getRoute()), 0.06);
}
BOOST_AUTO_TEST_CASE(multi_distance_many_point_large_test)
{
	FMS fms;
	std::vector < std::tuple<double, double, double >> path;
	path.push_back(std::make_tuple(43.77109, -79.71680, 10)); // Starting Point
	path.push_back(std::make_tuple(22.83695, -82.44141, 10));
	path.push_back(std::make_tuple(33.94336, -118.30078, 10));
	path.push_back(std::make_tuple(51.78144, -125.50781, 10));
	path.push_back(std::make_tuple(43.96119, -79.18945, 10));
	fms.setRoute(path);
	BOOST_CHECK_CLOSE(11610.135, utility::toKilometer(calcTotalDistance(fms.getRoute())), 0.08);
}

BOOST_AUTO_TEST_CASE(multi_distance_one_point_test)
{
	FMS fms;
	route_t path;
	path.push_back(std::make_tuple(43.37999, -80.94842, 10)); // Starting Point
	fms.setRoute(path);
	BOOST_CHECK_EQUAL(0, calcTotalDistance(fms.getRoute()));
}
BOOST_AUTO_TEST_CASE(multi_distance_no_point_test)
{
	FMS fms;
	route_t path;
	fms.setRoute(path);
	BOOST_CHECK_EQUAL(0, calcTotalDistance(fms.getRoute()));
}

BOOST_AUTO_TEST_CASE(single_bearing_test)
{
	route_t path;
	BOOST_CHECK_CLOSE(24.54650859508968, calcBearing(std::make_tuple(43.012618126753544, -81.19999408721924, 10), std::make_tuple(43.37311218382002, -80.97335815429688, 10)), .1);
	BOOST_CHECK_CLOSE(57.36, calcBearing(std::make_tuple(40.76, -73.984, 10), std::make_tuple(41.89, 12.492, 10)), .1);
	//BOOST_CHECK_EQUAL(6886, utility::toKilometer(calcDistance(std::make_pair(40.76, -73.984), std::make_pair(41.89, 12.492))));
}

BOOST_AUTO_TEST_CASE(multi_bearing_test)
{
	FMS fms;
	route_t path;
	path.push_back(std::make_tuple(43.77109, -79.71680, 10)); // Starting Point
	path.push_back(std::make_tuple(22.83695, -82.44141, 10));
	path.push_back(std::make_tuple(33.94336, -118.30078, 10));
	path.push_back(std::make_tuple(51.78144, -125.50781, 10));
	path.push_back(std::make_tuple(43.96119, -79.18945, 10));
	fms.setRoute(path);
	std::vector<double> bearing = calcTripBearing(path);
	std::vector<double> correctBearing;

	correctBearing.push_back(187.0325);
	correctBearing.push_back(297.47201);
	correctBearing.push_back(345.8481);
	correctBearing.push_back(85.68785);
	for (size_t i = 0; i < correctBearing.size(); ++i)
		BOOST_CHECK_CLOSE(bearing[i], correctBearing[i], 0.1);
}

BOOST_AUTO_TEST_CASE(waypoint_error_thrown)
{
	FMS fms;
	BOOST_CHECK_THROW(fms.nextWaypoint(), error);
	BOOST_CHECK_THROW(fms.setWaypoint(6), error);
	route_t path;
	path.push_back(std::make_tuple(1, 1, 10));
	path.push_back(std::make_tuple(2, 2, 10));
	path.push_back(std::make_tuple(3, 3, 10));
	fms.setRoute(path);
	BOOST_CHECK_NO_THROW(fms.nextWaypoint());
	BOOST_CHECK_NO_THROW(fms.setWaypoint(3));
	BOOST_CHECK_THROW(fms.nextWaypoint(), error);
	BOOST_CHECK_THROW(fms.setWaypoint(-1), error);
	fms.setWaypoint(0);
	for (size_t i = 0; i < path.size(); ++i)
		BOOST_CHECK_NO_THROW(fms.nextWaypoint());
}
BOOST_AUTO_TEST_CASE(bearing_error_thrown)
{
	FMS fms;
	BOOST_CHECK_THROW(fms.setHeading(361), error);
	BOOST_CHECK_THROW(fms.setHeading(-1), error);
	BOOST_CHECK_NO_THROW(fms.setHeading(360));
	BOOST_CHECK_NO_THROW(fms.setHeading(0));
}
BOOST_AUTO_TEST_CASE(bearing_distance_function)
{
	FMS fms;
	std::vector < std::tuple<double, double, double >> path;
	path.push_back(std::make_tuple(43.67631, -79.63440, 10));
	fms.setLoc(std::make_tuple(43.38110, -80.95924, 10));
	fms.setRoute(path);
	fms.setWaypointBearing();
	fms.setWaypointDistance();
	BOOST_CHECK_CLOSE(fms.getWaypointBearing(), 72.517, 0.1);
	BOOST_CHECK_CLOSE(utility::toKilometer(fms.getWaypointDistance()), 112.006, 0.5);
}
BOOST_AUTO_TEST_CASE(setLoc_out_of_range)
{
	FMS fms;
	//Positives
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(91, 100, 10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(90.1, 100, 10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(80, 181, 10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(80, 180.1, 10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(90.1, 180.1, 10)), error);

	//Negitives
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-91, -100, 10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-90.1, -100, 10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-80, -181, 10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-80, -180.1, 10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-90.1, -180.1, 10)), error);
}
BOOST_AUTO_TEST_CASE(setLoc_in_range)
{
	FMS fms;
	//Positives
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(45, 23, 10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(75, 43, 10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(80, 123, 10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(23, 163, 10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(33, 34, 10)));

	//Negitives
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-45, -23, 10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-75, -43, 10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-80, -123, 10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-23, -163, 10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-33, -34, 10)));
}
BOOST_AUTO_TEST_CASE(clear_route)
{
	FMS fms;
	route_t path;
	path.push_back(std::make_tuple(22.83695, -82.44141, 10));
	path.push_back(std::make_tuple(33.94336, -118.30078, 10));
	path.push_back(std::make_tuple(51.78144, -125.50781, 10));
	path.push_back(std::make_tuple(43.96119, -79.18945, 10));
	fms.setRoute(path);
	BOOST_CHECK_EQUAL(fms.getRoute().size(), 4);
	fms.clearRoute();
	BOOST_CHECK_EQUAL(fms.getRoute().size(), 0);
}
BOOST_AUTO_TEST_CASE(fms_one_arg_class_constructor)
{
	route_t path;
	path.push_back(std::make_tuple(22.83695, -82.44141, 10));
	path.push_back(std::make_tuple(33.94336, -118.30078, 10));
	path.push_back(std::make_tuple(51.78144, -125.50781, 10));
	path.push_back(std::make_tuple(43.96119, -79.18945, 10));
	BOOST_CHECK_NO_THROW(FMS fms(path));
	FMS fms(path);
	BOOST_CHECK_EQUAL(fms.getRoute().size(), 4);
	for (route_t::iterator it = path.begin(); it != path.end(); ++it)
		BOOST_CHECK(*it == fms.getRoute()[std::distance(path.begin(), it)]);
}
BOOST_AUTO_TEST_CASE(route_lengths_less_than_min_size_apart)
{
	route_t path;
	// Same position
	path.push_back(std::make_tuple(22.83695, -82.44141, 10));
	path.push_back(std::make_tuple(22.83695, -82.44141, 10));
	BOOST_CHECK_THROW(FMS fms(path), error);

	path.clear();
	// 15 meters apart
	path.push_back(std::make_tuple(43.37973, -80.96051, 10));
	path.push_back(std::make_tuple(43.37971, -80.96032, 10));
	BOOST_CHECK_NO_THROW(FMS fms(path));

	path.clear();
	// 8 meters apart
	path.push_back(std::make_tuple(43.37973, -80.96051, 10));
	path.push_back(std::make_tuple(43.37973, -80.96041, 10));
	BOOST_CHECK_THROW(FMS fms(path), error);
}
BOOST_AUTO_TEST_CASE(route_insert_waypoint)
{
	route_t path;
	// Route with 4 waypoints
	path.push_back(std::make_tuple(22.83695, -82.44141, 10));
	path.push_back(std::make_tuple(33.94336, -118.30078, 10));
	path.push_back(std::make_tuple(51.78144, -125.50781, 10));
	path.push_back(std::make_tuple(43.96119, -79.18945, 10));
	FMS fms(path);

	BOOST_CHECK_THROW(fms.insertWaypoint(std::make_tuple(22.83695, -82.44141, 10), -1), error);
	BOOST_CHECK_THROW(fms.insertWaypoint(std::make_tuple(22.83695, -82.44141, 10), 6), error);
	BOOST_CHECK_NO_THROW(fms.insertWaypoint(std::make_tuple(10.83695, -81.44141, 10), 4));
	BOOST_CHECK_NO_THROW(fms.insertWaypoint(std::make_tuple(10.83695, -81.44141, 10), 0));

	BOOST_CHECK_NO_THROW(fms.insertWaypoint(std::make_tuple(33.94336, -118.30065, 10), 2)); // Distance of 12m from NEXT
	BOOST_CHECK_THROW(fms.insertWaypoint(std::make_tuple(33.94336, -118.30069, 10), 2), error); // Distance of 8m from NEXT

	BOOST_CHECK_NO_THROW(fms.insertWaypoint(std::make_tuple(22.83695, -82.44152, 10), 2)); // Distance of 12m from BEFORE
	BOOST_CHECK_THROW(fms.insertWaypoint(std::make_tuple(22.83695, -82.44133, 10), 2), error); // Distance of 8m from BEFORE

	BOOST_CHECK_NO_THROW(fms.insertWaypoint(std::make_tuple(10.83695, -81.44152, 10), 0)); // Distance of 12m
	BOOST_CHECK_THROW(fms.insertWaypoint(std::make_tuple(10.83695, -81.44159, 10), 0), error); // Distance of 8m
}

BOOST_AUTO_TEST_CASE(route_remove_waypoint)
{
	route_t path;
	// Route with 4 waypoints
	path.push_back(std::make_tuple(22.83695, -82.44141, 10));
	path.push_back(std::make_tuple(33.94336, -118.30078, 10)); // Original
	path.push_back(std::make_tuple(51.78144, -125.50781, 10));
	path.push_back(std::make_tuple(33.94336, -118.30091, 10)); // 12 meters
	path.push_back(std::make_tuple(43.96119, -79.18945, 10));
	path.push_back(std::make_tuple(33.94336, -118.30100, 10)); // 8 meters
	FMS fms(path);

	// Outside range
	BOOST_CHECK_THROW(fms.removeWaypoint(6), error);
	BOOST_CHECK_THROW(fms.removeWaypoint(-1), error);

	// Inside range

	BOOST_CHECK_EQUAL((fms.getRoute()).size(), 6);
	BOOST_CHECK_NO_THROW(fms.removeWaypoint(2));
	BOOST_CHECK_EQUAL((fms.getRoute()).size(), 5);
	BOOST_CHECK_THROW(fms.removeWaypoint(3), error);
	BOOST_CHECK_EQUAL((fms.getRoute()).size(), 5);

	fms.clearRoute();
	path.clear();
	path.push_back(std::make_tuple(22.83695, -82.44141, 10));
	path.push_back(std::make_tuple(33.94336, -118.30078, 10));
	path.push_back(std::make_tuple(51.78144, -125.50781, 10));
	path.push_back(std::make_tuple(33.94336, -118.30091, 10));
	fms.setRoute(path);
	BOOST_CHECK_EQUAL((fms.getRoute()).size(), path.size());
	BOOST_CHECK_NO_THROW(fms.removeWaypoint(3));
	BOOST_CHECK(fms.getRoute()[2] == path[2]);
	BOOST_CHECK(fms.getRoute()[2] == path[2]);
	BOOST_CHECK_NO_THROW(fms.removeWaypoint(0));
	BOOST_CHECK(fms.getRoute()[0] == path[1]);
	BOOST_CHECK(fms.getRoute()[1] == path[2]);

	/*utility::printRoutePositions(fms.getRoute());
	std::cout << std::endl;*/
}