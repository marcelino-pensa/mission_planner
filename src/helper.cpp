
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

geometry_msgs::Point eigenvec2rospoint(Eigen::Vector3d vec) {
		geometry_msgs::Point Pt;
	Pt.x = vec[0]; Pt.y = vec[1]; Pt.z = vec[2];
	return Pt;
}

Eigen::Vector3d rostfvec2eigenvec(tf::Vector3 vec) {
	return Eigen::Vector3d(vec.x(), vec.y(), vec.z());
}

geometry_msgs::Quaternion set_quat(double roll, double pitch, double yaw) {
	tf::Quaternion quat;
	quat.setRPY(roll, pitch, yaw);
	return set_quat(quat.getW(), quat.getX(), quat.getY(), quat.getZ());
}

geometry_msgs::Quaternion set_quat(double w, double x, double y, double z) {
	geometry_msgs::Quaternion quat;
	quat.x = x; quat.y = y; quat.z = z; quat.w = w;
	return quat;
}


}  // namespace helper
