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
	BOOST_CHECK_CLOSE(820.5, vultron::calcDistance(std::make_tuple(43.38016054259229, -80.96304200426027,10), std::make_tuple(43.379957798085755, -80.95290325418398,10)), 0.1);
	// Test large distance
	BOOST_CHECK_CLOSE(10300000, vultron::calcDistance(std::make_tuple(43.38100270482399, -80.95941565767214,10), std::make_tuple(35.71073183384689, 139.7312071352967,10)), 0.1);

}
BOOST_AUTO_TEST_CASE(multi_distance_many_point_small_test)
{
	FMS fms;
	std::vector < std::tuple < double, double, double >> path;
	path.push_back(std::make_tuple(43.37999, -80.94842,10)); // Starting Point
	path.push_back(std::make_tuple(43.38019, -80.96310,10));
	path.push_back(std::make_tuple(43.38933, -80.96314,10));
	path.push_back(std::make_tuple(43.38939, -80.96744,10));
	path.push_back(std::make_tuple(43.40792, -80.95276,10));
	path.push_back(std::make_tuple(43.38104, -80.95920,10));
	fms.setRoute(path);
	BOOST_CHECK_CLOSE(7'966, calcTotalDistance(fms.getRoute()),0.1);
}
BOOST_AUTO_TEST_CASE(multi_distance_many_point_large_test)
{
	FMS fms;
	std::vector < std::tuple<double, double, double >> path;
	path.push_back(std::make_tuple(43.77109, -79.71680,10)); // Starting Point
	path.push_back(std::make_tuple(22.83695, -82.44141,10));
	path.push_back(std::make_tuple(33.94336, -118.30078,10));
	path.push_back(std::make_tuple(51.78144, -125.50781,10));
	path.push_back(std::make_tuple(43.96119, -79.18945,10));
	fms.setRoute(path);
	BOOST_CHECK_CLOSE(11'610'135, calcTotalDistance(fms.getRoute()), 0.1);
}

BOOST_AUTO_TEST_CASE(multi_distance_one_point_test)
{
	FMS fms;
	std::vector<std::tuple<double,double, double>> path;
	path.push_back(std::make_tuple(43.37999, -80.94842,10)); // Starting Point
	fms.setRoute(path);
	BOOST_CHECK_EQUAL(0, calcTotalDistance(fms.getRoute()));
}
BOOST_AUTO_TEST_CASE(multi_distance_no_point_test)
{
	FMS fms;
	std::vector<std::tuple<double,double, double>> path;
	fms.setRoute(path);
	BOOST_CHECK_EQUAL(0, calcTotalDistance(fms.getRoute()));
}

BOOST_AUTO_TEST_CASE(single_bearing_test)
{
	std::vector<std::tuple<double,double, double>> path;
	BOOST_CHECK_CLOSE(24.54650859508968, calcBearing(std::make_tuple(43.012618126753544, -81.19999408721924,10), std::make_tuple(43.37311218382002, -80.97335815429688,10)), .1);
	BOOST_CHECK_CLOSE(57.36, calcBearing(std::make_tuple(40.76, -73.984,10), std::make_tuple(41.89, 12.492,10)), .1);
	//BOOST_CHECK_EQUAL(6886, utility::toKilometer(calcDistance(std::make_pair(40.76, -73.984), std::make_pair(41.89, 12.492))));
}

BOOST_AUTO_TEST_CASE(multi_bearing_test)
{
	//FMS fms;
	//std::vector<std::pair<double, double>> path;
	//path.push_back(std::make_pair(43.77109, -79.71680)); // Starting Point
	//path.push_back(std::make_pair(22.83695, -82.44141));
	//path.push_back(std::make_pair(33.94336, -118.30078));
	//path.push_back(std::make_pair(51.78144, -125.50781));
	//path.push_back(std::make_pair(43.96119, -79.18945));
	//fms.setRoute(path);
	//std::vector<double> lengths = calcTripDistance(path);
	//std::vector<double> bearings = calcTripBearing(path);
	//std::cout << "Distances: " << std::endl;
	//for (std::vector<double>::iterator it = lengths.begin(); it != lengths.end(); ++it)
	//	std::cout << utility::toKilometer(*it) << std::endl;
	//std::cout << std::endl;
	//std::cout << "Bearings: " << std::endl;
	//for (std::vector<double>::iterator it = bearings.begin(); it != bearings.end(); ++it)
	//	std::cout << *it << std::endl;
	//std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(waypoint_error_thrown)
{
	FMS fms;
	BOOST_CHECK_THROW(fms.nextWaypoint(), error);
	BOOST_CHECK_THROW(fms.setWaypoint(6), error);
	std::vector<std::tuple<double,double, double>> path;
	path.push_back(std::make_tuple(1, 1,10));
	path.push_back(std::make_tuple(1, 1,10));
	path.push_back(std::make_tuple(1, 1,10));
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
BOOST_AUTO_TEST_CASE(bearing_distance_function)
{
	FMS fms;
	std::vector < std::tuple<double, double, double >> path;
	path.push_back(std::make_tuple(43.67631, -79.63440,10));
	fms.setLoc(std::make_tuple(43.38110,-80.95924,10));
	fms.setRoute(path);
	setCurrentBearing(fms);
	setCurrentDistance(fms);
	BOOST_CHECK_CLOSE(fms.getBearing(), 72.517,0.1);
	BOOST_CHECK_CLOSE(utility::toKilometer(fms.getWaypointDistance()), 112.006,0.5);
}
BOOST_AUTO_TEST_CASE(setLoc_out_of_range)
{
	FMS fms;
	//Positives
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(91, 100,10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(90.1, 100, 10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(80, 181,10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(80, 180.1,10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(90.1, 180.1,10)), error);

	//Negitives
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-91, -100,10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-90.1, -100,10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-80, -181,10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-80, -180.1,10)), error);
	BOOST_CHECK_THROW(fms.setLoc(std::make_tuple(-90.1, -180.1,10)), error);
}
BOOST_AUTO_TEST_CASE(setLoc_in_range)
{
	FMS fms;
	//Positives
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(45, 23,10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(75, 43,10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(80, 123,10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(23, 163,10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(33, 34,10)));

	//Negitives
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-45, -23,10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-75, -43,10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-80, -123,10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-23, -163,10)));
	BOOST_CHECK_NO_THROW(fms.setLoc(std::make_tuple(-33, -34,10)));
}
BOOST_AUTO_TEST_CASE(clear_route)
{
	FMS fms;
	std::vector<std::tuple<double,double, double>> path;
	path.push_back(std::make_tuple(22.83695, -82.44141,10));
	path.push_back(std::make_tuple(33.94336, -118.30078,10));
	path.push_back(std::make_tuple(51.78144, -125.50781,10));
	path.push_back(std::make_tuple(43.96119, -79.18945,10));
	fms.setRoute(path);
	BOOST_CHECK_EQUAL(fms.getRoute().size(), 4);
	fms.clearRoute();
	BOOST_CHECK_EQUAL(fms.getRoute().size(),0);
}
//BOOST_AUTO_TEST_CASE(set_route)
//{
//	FMS fms;
//	std::vector<std::pair<double, double>> path;
//	path.push_back(std::make_pair(25.6654, 82.44141));
//	path.push_back(std::make_pair(33.94336, -118.30078));
//	path.push_back(std::make_pair(51.78144, 125.8854));
//	path.push_back(std::make_pair(43.96119, -79.18945));
//	fms.setRoute(path);
//	for (std::vector<std::pair<double, double>>::iterator it = (fms.getRoute()).begin(); it != (fms.getRoute()).end(); ++it)
//		BOOST_CHECK(*(it) == path[std::distance((fms.getRoute()).begin(), it)]);
//
//}