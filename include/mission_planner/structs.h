
#ifndef MISSION_STRUCTS_H_
#define MISSION_STRUCTS_H_

#include <mission_planner/helper.h>
#include <mutex>
#include <queue>
#include <Eigen/Dense>

#include "tf/tf.h"
#include <actionlib_msgs/GoalStatusArray.h>
#include "mg_msgs/PVAJS_array.h"

namespace mission_planner {

class xyz_heading {
 public:
	float x_;
	float y_;
	float z_;
	float yaw_;

	xyz_heading() {};

	xyz_heading(double x, double y, double z, double yaw) {
		x_ = x; y_ = y; z_ = z; yaw_ = yaw;
	}

	xyz_heading(Eigen::Vector3d xyz, double yaw) {
		x_ = xyz[0]; y_ = xyz[1]; z_ = xyz[2]; yaw_ = yaw;
	}

	xyz_heading(tf::StampedTransform transform) {
		Eigen::Vector3d pos = helper::rostfvec2eigenvec(transform.getOrigin());
		double roll, pitch, yaw;
		transform.getBasis().getRPY(roll, pitch, yaw);
		x_ = pos[0]; y_ = pos[1]; z_ = pos[2]; yaw_ = yaw;
	}

	geometry_msgs::Point GetXYZ() const {
		geometry_msgs::Point Pt;
		Pt.x = x_; Pt.y = y_; Pt.z = z_;
		return Pt;
	}

	double GetYaw() const {
		return yaw_;
	}

	Eigen::Vector3d GetEigenXYZ() const {
		return Eigen::Vector3d(x_, y_, z_);
	}

	double GetXYZDist(Eigen::Vector3d vector2) const {
		return (this->GetEigenXYZ()  - vector2).norm();
	}

	double GetXYZDist(xyz_heading vector2) const {
		return (this->GetEigenXYZ()  - vector2.GetEigenXYZ()).norm();
	}

	// void transformWaypoints(tf::StampedTransform pose) {
	// 	tf::Vector3 pos(x_, y_, z_);
	// }
};

class minSnapWpInputs {
 public:
	std::vector<xyz_heading> waypoints_;
	Eigen::Vector3d init_vel_;
	Eigen::Vector3d final_vel_;
	double max_vel_;
	double max_acc_;
	double sampling_time_;

	minSnapWpInputs() {};
	minSnapWpInputs(const std::vector<xyz_heading> &waypoints, const Eigen::Vector3d &init_vel,
		            const Eigen::Vector3d &final_vel, const double &max_vel, const double &max_acc,
		            const double &sampling_time) {
		waypoints_ = waypoints;
		init_vel_ = init_vel;
		final_vel_ = final_vel;
		max_vel_ = max_vel;
		max_acc_ = max_acc;
		sampling_time_ = sampling_time;
	}
};

struct TrajectoryActionInputs {
	mg_msgs::PVAJS_array flatStates;
	double sampling_time;
	bool start_immediately = false;
};

// Mutex protected variables
struct globalVariables {    
    tf::StampedTransform tf_quad2world;
    actionlib_msgs::GoalStatusArray action_server_status;

    // Buffer for solving minimum snap problems
	std::queue<minSnapWpInputs> min_snap_inputs;

	// Buffer for sending trajectories to the quad
	std::list<TrajectoryActionInputs> traj_inputs;
};

struct mutexStruct {
    std::mutex tf;
    std::mutex action_server_status;
    std::mutex waypoint_buffer;
    std::mutex trajectory_buffer;
};


}  // namespace mission_planner

#endif  // MISSION_STRUCTS_H_