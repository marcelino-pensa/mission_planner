
#ifndef MISSION_DRONE_STRUCTS_H_
#define MISSION_DRONE_STRUCTS_H_

#include <mission_planner/helper.h>
#include <mission_planner/xyz_heading.h>
#include <mutex>
#include <queue>
#include <Eigen/Dense>

#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/Path.h>
#include "tf/tf.h"
#include "mg_msgs/PVAJ_request.h"
#include "mg_msgs/PVAJS_array.h"

namespace mission_planner {

class minSnapWpInputs {
 public:
	std::vector<xyz_heading> waypoints_;
	Eigen::Vector3d init_vel_;
	Eigen::Vector3d final_vel_;
	double max_vel_;
	double max_acc_;
	double sampling_time_;
	std::string name_;

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
		name_ = "";
	}
	minSnapWpInputs(const std::vector<xyz_heading> &waypoints, const Eigen::Vector3d &init_vel,
		            const Eigen::Vector3d &final_vel, const double &max_vel, const double &max_acc,
		            const double &sampling_time, const std::string &name) {
		waypoints_ = waypoints;
		init_vel_ = init_vel;
		final_vel_ = final_vel;
		max_vel_ = max_vel;
		max_acc_ = max_acc;
		sampling_time_ = sampling_time;
		name_ = name;
	}

	// Create an action of Halt (name = "Halt") or Disarm (name = "Disarm")
	minSnapWpInputs(const std::string name) {
		name_ = name;
	}
};

class minTimeWpInputs {
 public:
	std::vector<xyz_heading> waypoints_;
	double max_vel_;
	double max_acc_;
	double max_jerk_;
	double sampling_time_;
	std::string name_;

	minTimeWpInputs() {};
	minTimeWpInputs(const std::vector<xyz_heading> &waypoints, const double &max_vel,
					const double &max_acc, const double &max_jerk, const double &sampling_time) {
		waypoints_ = waypoints;
		max_vel_ = max_vel;
		max_acc_ = max_acc;
		max_jerk_ = max_jerk;
		sampling_time_ = sampling_time;
		name_ = "";
	}
	minTimeWpInputs(const std::vector<xyz_heading> &waypoints, const double &max_vel,
		            const double &max_acc, const double &max_jerk,
		            const double &sampling_time, const std::string &name) {
		waypoints_ = waypoints;
		max_vel_ = max_vel;
		max_acc_ = max_acc;
		max_jerk_ = max_jerk;
		sampling_time_ = sampling_time;
		name_ = name;
	}

	// Create an action of Halt (name = "Halt") or Disarm (name = "Disarm")
	minTimeWpInputs(const std::string name) {
		name_ = name;
	}
};

// Class used for visualizing trajectories
class waypoint_and_trajectory {
 public:
  nav_msgs::Path Waypoints_;
  mg_msgs::PVAJS_array flatStates_;
  std::string traj_name_;

  waypoint_and_trajectory() {}
  waypoint_and_trajectory(nav_msgs::Path Waypoints,
                          mg_msgs::PVAJS_array flatStates,
                          std::string traj_name = "") {
    Waypoints_ = Waypoints;
    flatStates_ = flatStates;
    traj_name_ = traj_name;
  }
  waypoint_and_trajectory(std::vector<xyz_heading> xyz_heading_array,
                          mg_msgs::PVAJS_array flatStates,
                          std::string traj_name = "") {
    for (uint i = 0; i < xyz_heading_array.size(); i++) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position = xyz_heading_array[i].GetXYZ();
      pose.pose.orientation.w  = cos(xyz_heading_array[i].GetYaw()/2.0);
      pose.pose.orientation.z  = sin(xyz_heading_array[i].GetYaw()/2.0);
      Waypoints_.poses.push_back(pose);
    }
    flatStates_ = flatStates;
    traj_name_ = traj_name;
  }
};

enum class ActionType {Trajectory, Halt, Disarm};

struct TrajectoryActionInputs {
	mg_msgs::PVAJS_array flatStates;
	double sampling_time;
	bool start_immediately = false;
	ActionType action_type;
};

// Mutex protected variables
struct globalVariables {    
    tf::StampedTransform tf_quad2world;
    actionlib_msgs::GoalStatusArray action_server_status;

    // Buffer for solving minimum snap problems
	std::queue<minSnapWpInputs> min_snap_inputs;

	// Buffer for solving minimum time problems
	std::queue<minTimeWpInputs> min_time_inputs;

	// Buffer for sending trajectories to the quad
	std::list<TrajectoryActionInputs> traj_inputs;

	// List of waypoint and respective trajectories for rviz visualization
	std::queue<waypoint_and_trajectory> wp_traj_list;

	// Flag to determine whether quad is busy with some action
  	bool quad_is_busy;
};

struct mutexStruct {
    std::mutex tf;
    std::mutex action_server_status;
    std::mutex waypoint_buffer;
    std::mutex trajectory_buffer;
    std::mutex wp_traj_buffer;
    std::mutex quad_is_busy;
};


}  // namespace mission_planner

#endif  // MISSION_DRONE_STRUCTS_H_