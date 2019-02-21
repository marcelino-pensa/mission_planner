
#ifndef MISSION_STRUCTS_H_
#define MISSION_STRUCTS_H_

#include <mission_planner/helper.h>
#include <mission_planner/xyz_heading.h>
#include <mutex>
#include <queue>
#include <Eigen/Dense>

#include "tf/tf.h"
#include <actionlib_msgs/GoalStatusArray.h>
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

	// Buffer for sending trajectories to the quad
	std::list<TrajectoryActionInputs> traj_inputs;

	// Flag to determine whether quad is busy with some action
  	bool quad_is_busy;
};

struct mutexStruct {
    std::mutex tf;
    std::mutex action_server_status;
    std::mutex waypoint_buffer;
    std::mutex trajectory_buffer;
    std::mutex quad_is_busy;
};


}  // namespace mission_planner

#endif  // MISSION_STRUCTS_H_