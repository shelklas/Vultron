#define BOOST_TEST_MODULE VultronUnitTest
#include "fms.h"
#include <boost/test/auto_unit_test.hpp>
#include <vector>
#include <iostream>
#include "utility.h"

using namespace vultron;
BOOST_AUTO_TEST_CASE(single_distance_test)
{
	// Test small distance
	BOOST_CHECK_CLOSE(820.5, vultron::calcDistance(std::make_pair(43.38016054259229, -80.96304200426027), std::make_pair(43.379957798085755, -80.95290325418398)), 0.1);
	// Test large distance
	BOOST_CHECK_CLOSE(10300000, vultron::calcDistance(std::make_pair(43.38100270482399, -80.95941565767214), std::make_pair(35.71073183384689, 139.7312071352967)), 0.1);

}
BOOST_AUTO_TEST_CASE(multi_distance_many_point_small_test)
{
	FMS fms;
	std::vector<std::pair<double, double>> path;
	path.push_back(std::make_pair(43.37999, -80.94842)); // Starting Point
	path.push_back(std::make_pair(43.38019, -80.96310));
	path.push_back(std::make_pair(43.38933, -80.96314));
	path.push_back(std::make_pair(43.38939, -80.96744));
	path.push_back(std::make_pair(43.40792, -80.95276));
	path.push_back(std::make_pair(43.38104, -80.95920));
	fms.setRoute(path);
	BOOST_CHECK_CLOSE(7'966, calcTotalDistance(fms.getRoute()),0.1);
}
BOOST_AUTO_TEST_CASE(multi_distance_many_point_large_test)
{
	FMS fms;
	std::vector<std::pair<double, double>> path;
	path.push_back(std::make_pair(43.77109, -79.71680)); // Starting Point
	path.push_back(std::make_pair(22.83695, -82.44141));
	path.push_back(std::make_pair(33.94336, -118.30078));
	path.push_back(std::make_pair(51.78144, -125.50781));
	path.push_back(std::make_pair(43.96119, -79.18945));
	fms.setRoute(path);
	BOOST_CHECK_CLOSE(11'610'135, calcTotalDistance(fms.getRoute()), 0.1);
}

BOOST_AUTO_TEST_CASE(multi_distance_one_point_test)
{
	FMS fms;
	std::vector<std::pair<double, double>> path;
	path.push_back(std::make_pair(43.37999, -80.94842)); // Starting Point
	fms.setRoute(path);
	BOOST_CHECK_EQUAL(0, calcTotalDistance(fms.getRoute()));
}
BOOST_AUTO_TEST_CASE(multi_distance_no_point_test)
{
	FMS fms;
	std::vector<std::pair<double, double>> path;
	fms.setRoute(path);
	BOOST_CHECK_EQUAL(0, calcTotalDistance(fms.getRoute()));
}

BOOST_AUTO_TEST_CASE(single_bearing_test)
{
	std::vector<std::pair<double, double>> path;
	BOOST_CHECK_CLOSE(24.54650859508968, calcBearing(std::make_pair(43.012618126753544, -81.19999408721924), std::make_pair(43.37311218382002, -80.97335815429688)), .1);
	BOOST_CHECK_CLOSE(57.36, calcBearing(std::make_pair(40.76, -73.984), std::make_pair(41.89, 12.492)), .1);
	//BOOST_CHECK_EQUAL(6886, utility::toKilometer(calcDistance(std::make_pair(40.76, -73.984), std::make_pair(41.89, 12.492))));
}

BOOST_AUTO_TEST_CASE(multi_bearing_test)
{
	
}

BOOST_AUTO_TEST_CASE(waypoint_error_thrown)
{
	FMS fms;
	BOOST_CHECK_THROW(fms.nextWaypoint(), error);
	BOOST_CHECK_THROW(fms.setWaypoint(6), error);
	std::vector<std::pair<double, double>> path;
	path.push_back(std::make_pair(1, 1));
	path.push_back(std::make_pair(1, 1));
	path.push_back(std::make_pair(1, 1));
	fms.setRoute(path);
	BOOST_CHECK_NO_THROW(fms.nextWaypoint());
	BOOST_CHECK_NO_THROW(fms.setWaypoint(3));
	BOOST_CHECK_THROW(fms.nextWaypoint(), error);
	BOOST_CHECK_THROW(fms.setWaypoint(-1),error);
	fms.setWaypoint(0);
	for (size_t i = 0; i < path.size(); ++i)
		BOOST_CHECK_NO_THROW(fms.nextWaypoint());
}
BOOST_AUTO_TEST_CASE(bearing_error_thrown)
{
	FMS fms;
	BOOST_CHECK_THROW(fms.setBearing(361), error);
	BOOST_CHECK_THROW(fms.setBearing(-1), error);
	BOOST_CHECK_NO_THROW(fms.setBearing(360));
	BOOST_CHECK_NO_THROW(fms.setBearing(0));
}