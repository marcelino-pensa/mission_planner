
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
#include "mission_planner/polynomials.h"
#include "mg_msgs/PolyPVA.h"
#include "mg_msgs/PVA_request.h"
#include "mg_msgs/PVA_array.h"

namespace mission_planner {

class minAccWpInputs {
 public:
	std::vector<Eigen::Vector2d> waypoints_;
	Eigen::Vector2d init_vel_;
	Eigen::Vector2d final_vel_;
	double max_vel_;
	double max_acc_;
	std::string name_;

	minAccWpInputs() {};
	minAccWpInputs(const std::vector<Eigen::Vector2d> &waypoints, const Eigen::Vector2d &init_vel,
		           const Eigen::Vector2d &final_vel, const double &max_vel, const double &max_acc) {
		waypoints_ = waypoints;
		init_vel_ = init_vel;
		final_vel_ = final_vel;
		max_vel_ = max_vel;
		max_acc_ = max_acc;
		name_ = "";
	}
	minAccWpInputs(const std::vector<Eigen::Vector2d> &waypoints, const Eigen::Vector2d &init_vel,
		           const Eigen::Vector2d &final_vel, const double &max_vel, const double &max_acc, 
		           const std::string &name) {
		waypoints_ = waypoints;
		init_vel_ = init_vel;
		final_vel_ = final_vel;
		max_vel_ = max_vel;
		max_acc_ = max_acc;
		name_ = name;
	}

	// Create an action of Halt (name = "Halt") or Disarm (name = "Disarm")
	minAccWpInputs(const std::string name) {
		name_ = name;
	}
};

// Class used for visualizing trajectories
class waypoint_and_trajectory {
 public:
  std::vector<Eigen::Vector2d> waypoints_;
  tucker_polynomials::Trajectory2D trajectory_;
  std::string traj_name_;

  waypoint_and_trajectory() {}
  // Still need to implement the constructor below
  waypoint_and_trajectory(std::vector<Eigen::Vector2d> xy_array,
                          std::vector<mg_msgs::PolyPVA> &polyX,
                          std::vector<mg_msgs::PolyPVA> &polyY,
                          std::string traj_name = "") {
    waypoints_ = xy_array;
    trajectory_ = tucker_polynomials::Trajectory2D(polyX, polyY);
    traj_name_ = traj_name;
  }
};

enum class ActionType {Trajectory, Halt};

struct TrajectoryActionInputs {
	std::vector<mg_msgs::PolyPVA> polyX;
    std::vector<mg_msgs::PolyPVA> polyY;
	bool start_immediately = false;
	ActionType action_type;
};

// Mutex protected variables
struct globalVariables {    
    tf::StampedTransform tf_rover2world;
    actionlib_msgs::GoalStatusArray action_server_status;

    // Buffer for solving minimum snap problems
	std::queue<minAccWpInputs> min_acc_inputs;

	// Buffer for sending trajectories to the quad
	std::list<TrajectoryActionInputs> traj_inputs;

	// List of waypoint and respective trajectories for rviz visualization
	std::queue<waypoint_and_trajectory> wp_traj_list;

	// Flag to determine whether rover is busy with some action
  	bool rover_is_busy;
};

struct mutexStruct {
    std::mutex tf;
    std::mutex action_server_status;
    std::mutex waypoint_buffer;
    std::mutex trajectory_buffer;
    std::mutex wp_traj_buffer;
    std::mutex rover_is_busy;
};


}  // namespace mission_planner

#endif  // MISSION_DRONE_STRUCTS_H_