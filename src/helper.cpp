
// Standard includes
#include <inspector/helper.h>

namespace helper {

double deg2rad(double deg) {
	return deg*M_PI/180.0;
}

double rad2deg(double rad) {
	return rad*180.0/M_PI;
}

geometry_msgs::Point rosvec2rospoint(geometry_msgs::Vector3 vec) {
	geometry_msgs::Point Pt;
	Pt.x = vec.x; Pt.y = vec.y; Pt.z = vec.z;
	return Pt;
}

geometry_msgs::Point rostfvec2rospoint(tf::Vector3 vec) {
	geometry_msgs::Point Pt;
	Pt.x = vec.x(); Pt.y = vec.y(); Pt.z = vec.z();
	return Pt;
}


}  // namespace helper
