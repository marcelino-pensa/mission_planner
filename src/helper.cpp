
// Standard includes
#include <mission_planner/helper.h>

namespace helper {

double deg2rad(const double &deg) {
	return deg*M_PI/180.0;
}

double rad2deg(const double &rad) {
	return rad*180.0/M_PI;
}

geometry_msgs::Point set_rospoint(const double &x, const double &y, const double &z) {
	geometry_msgs::Point pt;
	pt.x = x; pt.y = y; pt.z = z;
	return pt;
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

geometry_msgs::Point eigenvec2d2rospoint(const Eigen::Vector2d &vec) {
	geometry_msgs::Point Pt;
	Pt.x = vec[0]; Pt.y = vec[1]; Pt.z = 0.0;
	return Pt;
}

geometry_msgs::Point eigenvec2d2rospoint(const Eigen::Vector2d &vec,
	                                     const double height) {
	geometry_msgs::Point Pt;
	Pt.x = vec[0]; Pt.y = vec[1]; Pt.z = height;
	return Pt;
}

geometry_msgs::Vector3 setvector3(const double &x, const double &y, const double &z) {
	geometry_msgs::Vector3 vec3;
	vec3.x = x; vec3.y = y; vec3.z = z;
	return vec3;
}

geometry_msgs::Vector3 eigenvec2rosvec(const Eigen::Vector3d &vec) {
	geometry_msgs::Vector3 vec3;
	vec3.x = vec[0]; vec3.y = vec[1]; vec3.z = vec[2];
	return vec3;
}

geometry_msgs::Vector3 eigenvec2rosvec(const Eigen::Vector2d &vec) {
	geometry_msgs::Vector3 vec3;
	vec3.x = vec[0]; vec3.y = vec[1]; vec3.z = 0.0;
	return vec3;
}

Eigen::Vector3d rostfvec2eigenvec(const tf::Vector3 &vec) {
	return Eigen::Vector3d(vec.x(), vec.y(), vec.z());
}

Eigen::Vector3d rospoint2eigenvec(const geometry_msgs::Point & p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Quaterniond ros2eigenquat(const geometry_msgs::Quaternion & q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
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

Eigen::Vector3d quat2rpy(geometry_msgs::Quaternion quat) {
	double qx, qy, qz, qw, roll, pitch, yaw;
	qx = quat.x;
	qy = quat.y;
	qz = quat.z;
	qw = quat.w;

	//Formulas for roll, pitch, yaw
	roll = atan2(2*(qw*qx + qy*qz) , 1 - 2*(qx*qx + qy*qy) );
	pitch = asin(2*(qw*qy - qz*qx));
	yaw = atan2(2*(qw*qz + qx*qy),1 - 2*(qy*qy + qz*qz) );

	Eigen::Vector3d rpy(roll, pitch, yaw);
	return rpy;
}

double getHeadingFromQuat(geometry_msgs::Quaternion quat) {
	Eigen::Vector3d RPY = quat2rpy(quat);
	return RPY[2];
}

double getHeadingFromTransform(tf::StampedTransform transform) {
	double roll, pitch, yaw;
	transform.getBasis().getRPY(roll, pitch, yaw);
	return yaw;
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

mg_msgs::PVA_request get_empty_PVA() {
	mg_msgs::PVA_request PVA;
	PVA.use_pos = false;
	PVA.use_vel = false;
	PVA.use_acc = false;
	PVA.use_yaw = false;
	PVA.use_yaw_dot = false;
	return PVA;
}

mg_msgs::PVAJS_array pva2pvajs(const std::vector<p4_ros::PVA> &PVA) {
	mg_msgs::PVAJS_array pvajs_array;
	for (uint i = 0; i < PVA.size(); i++) {
        mg_msgs::PVAJS new_flat_state;
        new_flat_state.Pos = PVA[i].pos;
        new_flat_state.Vel = PVA[i].vel;
        new_flat_state.Acc = PVA[i].acc;
        new_flat_state.time = PVA[i].time;
        new_flat_state.Jerk = helper::setvector3(0.0, 0.0, 0.0);
        new_flat_state.Snap = helper::setvector3(0.0, 0.0, 0.0);
        new_flat_state.yaw = 0.0;
        new_flat_state.yaw_dot = 0.0;
        new_flat_state.yaw_ddot = 0.0;
        pvajs_array.PVAJS_array.push_back(new_flat_state);
    }
    return pvajs_array;
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
