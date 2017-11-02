#define BOOST_TEST_MODULE VultronUnitTest
#include "calc.h"
#include <boost/test/auto_unit_test.hpp>
#include <vector>

BOOST_AUTO_TEST_CASE(single_distance_test)
{
	// Test small distance
	BOOST_CHECK_CLOSE(820.5, vultron::calcDistance(std::make_pair(43.38016054259229, -80.96304200426027), std::make_pair(43.379957798085755, -80.95290325418398)), 0.1);
	// Test large distance
	BOOST_CHECK_CLOSE(10300000, vultron::calcDistance(std::make_pair(43.38100270482399, -80.95941565767214), std::make_pair(35.71073183384689, 139.7312071352967)), 0.1);

}
BOOST_AUTO_TEST_CASE(multi_distance_many_point_small_test)
{
	std::vector<std::pair<double, double>> path;
	path.push_back(std::make_pair(43.37999, -80.94842)); // Starting Point
	path.push_back(std::make_pair(43.38019, -80.96310));
	path.push_back(std::make_pair(43.38933, -80.96314));
	path.push_back(std::make_pair(43.38939, -80.96744));
	path.push_back(std::make_pair(43.40792, -80.95276));
	path.push_back(std::make_pair(43.38104, -80.95920));
	BOOST_CHECK_CLOSE(7'966, vultron::calcDistance(path),0.1);
}
BOOST_AUTO_TEST_CASE(multi_distance_many_point_large_test)
{
	std::vector<std::pair<double, double>> path;
	path.push_back(std::make_pair(43.77109, -79.71680)); // Starting Point
	path.push_back(std::make_pair(22.83695, -82.44141));
	path.push_back(std::make_pair(33.94336, -118.30078));
	path.push_back(std::make_pair(51.78144, -125.50781));
	path.push_back(std::make_pair(43.96119, -79.18945));
	BOOST_CHECK_CLOSE(11'610'135, vultron::calcDistance(path), 0.01);
}

BOOST_AUTO_TEST_CASE(multi_distance_one_point_test)
{
	std::vector<std::pair<double, double>> path;
	path.push_back(std::make_pair(43.37999, -80.94842)); // Starting Point
	BOOST_CHECK_EQUAL(0, vultron::calcDistance(path));
}
BOOST_AUTO_TEST_CASE(multi_distance_no_point_test)
{
	std::vector<std::pair<double, double>> path;
	BOOST_CHECK_EQUAL(0, vultron::calcDistance(path));
}