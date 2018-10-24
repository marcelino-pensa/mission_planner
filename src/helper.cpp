
// Standard includes
#include <mission_planner/helper.h>

namespace helper {

double deg2rad(const double &deg) {
	return deg*M_PI/180.0;
}

double rad2deg(const double &rad) {
	return rad*180.0/M_PI;
}

geometry_msgs::Point rosvec2rospoint(const geometry_msgs::Vector3 &vec) {
	geometry_msgs::Point Pt;
	Pt.x = vec.x; Pt.y = vec.y; Pt.z = vec.z;
	return Pt;
}

geometry_msgs::Point rostfvec2rospoint(const tf::Vector3 &vec) {
	geometry_msgs::Point Pt;
	Pt.x = vec.x(); Pt.y = vec.y(); Pt.z = vec.z();
	return Pt;
}

geometry_msgs::Point eigenvec2rospoint(const Eigen::Vector3d &vec) {
	geometry_msgs::Point Pt;
	Pt.x = vec[0]; Pt.y = vec[1]; Pt.z = vec[2];
	return Pt;
}

geometry_msgs::Vector3 eigenvec2rosvec(const Eigen::Vector3d &vec) {
	geometry_msgs::Vector3 vec3;
	vec3.x = vec[0]; vec3.y = vec[1]; vec3.z = vec[2];
	return vec3;
}

Eigen::Vector3d rostfvec2eigenvec(const tf::Vector3 &vec) {
	return Eigen::Vector3d(vec.x(), vec.y(), vec.z());
}

geometry_msgs::Quaternion set_quat(const double &roll, const double &pitch, const double &yaw) {
	tf::Quaternion quat;
	quat.setRPY(roll, pitch, yaw);
	return set_quat(quat.getW(), quat.getX(), quat.getY(), quat.getZ());
}

geometry_msgs::Quaternion set_quat(const double &w, const double &x, const double &y, const double &z) {
	geometry_msgs::Quaternion quat;
	quat.x = x; quat.y = y; quat.z = z; quat.w = w;
	return quat;
}

mg_msgs::PVAJ_request get_empty_PVAJ() {
	mg_msgs::PVAJ_request PVAJ;
	PVAJ.use_pos = false;
	PVAJ.use_vel = false;
	PVAJ.use_acc = false;
	PVAJ.use_jerk = false;
	PVAJ.use_yaw = false;
	PVAJ.use_yaw_dot = false;
	return PVAJ;
}

std::vector<std::pair<uint, uint>> split_waypoints(const uint &n_waypoints,
	                                               const uint &wp_per_segment) {
	uint n_segments = std::ceil(double(n_waypoints-1.0)/double(wp_per_segment));
	std::vector<std::pair<uint, uint>> first_last_wp_index;
	// ROS_INFO("n_segments: %d", int(n_segments));
	for (uint i = 0; i < n_segments; i++) {
		uint first = i*wp_per_segment;
		uint last  = std::min((i+1)*wp_per_segment, n_waypoints-1);
		first_last_wp_index.push_back(std::pair<uint, uint>(first, last));
		// ROS_INFO("First: %d\tLast: %d", int(first), int(last));
	}

	return first_last_wp_index;
}


}  // namespace helper
