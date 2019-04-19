

#ifndef ROVER_MISSION_CLASS_H_
#define ROVER_MISSION_CLASS_H_

// Ros libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Local defined libraries
#include <mission_planner/rover_structs.h>
#include <mission_planner/helper.h>
#include <mission_planner/tf_class.h>
#include <mission_planner/visualization_functions.h>

// Msg/srv defined in other packages
#include "mg_msgs/minAccXYWpPVA.h"
#include "mg_msgs/PVA_request.h"

// Msg/srv types
#include "std_srvs/Trigger.h"

// ROS Action types
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <mg_msgs/follow_PolyPVA_XY_trajectoryAction.h>

// C++ libraries
#include <vector>
#include <string>
#include <thread>
#include <fstream>
#include <math.h>

// Eigen-based libraries
#include <Eigen/Dense>

namespace rover_planner {

class RoverMissionClass {
 public:
  RoverMissionClass() {};
  RoverMissionClass(const std::string &ns, const double &tf_update_rate,
               const double &max_velocity, const uint &rover_index = 0);
  ~RoverMissionClass();

  //Methods -------------------------------------------------------
  void Initialize(const std::string &ns, const double &tf_update_rate,
                  const double &max_velocity, const uint &rover_index = 0);

  // Helper function to add waypoints into the buffer
  void AddWaypoints2Buffer(const std::vector<Eigen::Vector2d> &waypoints, const Eigen::Vector2d &init_vel,  
                           const Eigen::Vector2d &final_vel, const double &max_vel, const double &max_acc,
                           const std::string &traj_name, Eigen::Vector2d *final_waypoint);

  // Returns current pose of the vehicle - thread safe
  tf::StampedTransform GetCurrentPose();

  // Wait until the first pose is obtained
  tf::StampedTransform WaitForFirstPose();

  // Method for getting a minimum acceleration trajectory between two points only
  bool MinAccPoint2Point(const std::string &ns, const Eigen::Vector2d &init_point,
                         const Eigen::Vector2d &final_point, const double &tf, ros::NodeHandle *nh,
                         std::vector<mg_msgs::PolyPVA> *polyX, std::vector<mg_msgs::PolyPVA> *polyY);

  // Method for getting minimum acceleration trajectories for multiple waypoints
  bool MinAccWaypointSet(const std::string &ns, const std::vector<Eigen::Vector2d> waypoints,
                         const Eigen::Vector2d &init_vel, const Eigen::Vector2d &final_vel,
                         const double &max_vel, const double &max_acc, ros::NodeHandle *nh,
                         std::vector<mg_msgs::PolyPVA> *polyX, std::vector<mg_msgs::PolyPVA> *polyY);

  // Different template for the function above
  bool MinAccWaypointSet(const std::string &ns, const rover_planner::minAccWpInputs &Inputs, ros::NodeHandle *nh,
                         std::vector<mg_msgs::PolyPVA> *polyX, std::vector<mg_msgs::PolyPVA> *polyY);

  // Calls different action types (Halt, disarm, PVAJS trajectory)
  void CallActionType(const std::string &ns, const rover_planner::TrajectoryActionInputs &traj_inputs, 
                      const bool &wait_until_done, ros::NodeHandle *nh,
                      actionlib::SimpleActionClient<mg_msgs::follow_PolyPVA_XY_trajectoryAction> *client);

  // Method for sending a PVAJS trajectory to the action server
  bool CallPVAAction(const std::string &ns, const rover_planner::TrajectoryActionInputs &traj_inputs,
                     const bool &wait_until_done, ros::NodeHandle *nh);

  // Similar to construction above, but the client is declared outside the function
  bool CallPVAAction(const std::string &ns, const rover_planner::TrajectoryActionInputs &traj_inputs,
                     const bool &wait_until_done, ros::NodeHandle *nh,
                     actionlib::SimpleActionClient<mg_msgs::follow_PolyPVA_XY_trajectoryAction> *client);

  // Returns whether rover is idle (not performing any actions)
  bool IsRoverIdle();

  // Blocking function that returns when quad is idle
  void ReturnWhenIdle();

  // Callbacks (see callbacks.cpp for implementation) ------------------------------
  void ActionGoalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);

 protected:
  // Threads for constantly updating the tfTree values
  void TfTask();

  // Thread that reads waypoints from a buffer to solve minimum snap problems
  void MinAccSolverTask(const std::string &ns);

  // Consumer thread
  // Consumes minimum snap trajectories (sends to action server)
  void TrajectoryActionCaller(const std::string &ns);

  // Thread for publishing trajectories into ROS
  void RvizPubThread(const std::string &ns);


 private:
  // Declare global variables (structures defined in structs.h)
  rover_planner::globalVariables globals_;  // These variables are all mutex-protected
  rover_planner::mutexStruct mutexes_;

  // Namespace of the current node
  std::string ns_;

  // Thread variables
  std::thread h_tf_thread_, h_min_acc_thread_, h_trajectory_caller_thread_, h_rviz_pub_thread_;

  // Subscriber variables
  ros::Subscriber action_status_sub_;

  // Thread rates (hz)
  double tf_update_rate_;

  // Navigation parameters
  double max_velocity_;

  // Rviz trajectory color
  std_msgs::ColorRGBA traj_color_;

};

}  // namespace rover_planner

#endif  // ROVER_MISSION_CLASS_H_