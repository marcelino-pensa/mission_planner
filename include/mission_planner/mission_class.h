

#ifndef MISSION_CLASS_H_
#define MISSION_CLASS_H_

// Ros libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Local defined libraries
#include <mission_planner/structs.h>
#include <mission_planner/helper.h>
#include <mission_planner/tf_class.h>

// Msg/srv defined in other packages
#include "mg_msgs/minSnapWpStamped.h"
#include "mg_msgs/minSnapWpPVAJ.h"
#include "mg_msgs/PVAJ_request.h"

// Msg/srv types
#include "std_srvs/Trigger.h"

// ROS Action types
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <mg_msgs/follow_PVAJS_trajectoryAction.h>

// C++ libraries
#include <vector>
#include <string>
#include <thread>
#include <fstream>
#include <math.h>

// Eigen-based libraries
#include <Eigen/Dense>

namespace mission_planner {

class MissionClass {
 public:
  MissionClass();
  ~MissionClass();

  // Method for executing a mission. This is implemented in different files, and then
  // the compiled one is executed
  virtual void Mission(ros::NodeHandle *nh);

 protected:
  // Callbacks (see callbacks.cpp for implementation) ----------------
  // Callback for handling incoming point cloud messages
  // void PclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg,
  //                  const uint& cam_index);

  // Method for loading waypoints from file
  bool LoadWaypoints(const std::string &filename,
                     const tf::StampedTransform &init_pose,
                     std::vector<xyz_heading> *waypoint_list);

  // Method for starting the quadcopter to listen to PVA messages
  void SetQuadPosMode(const std::string &ns, ros::NodeHandle *nh);

  // Method for disarming the quad (stop motors)
  void DisarmQuad(const std::string &ns, ros::NodeHandle *nh);

  // Helper function to add waypoints into the buffer
  void AddWaypoints2Buffer(const std::vector<xyz_heading> &waypoints, const Eigen::Vector3d &init_vel,
                           const Eigen::Vector3d &final_vel, const double &max_vel, const double &max_acc,
                           const double &sampling_time, xyz_heading *final_waypoint);

  // Method for taking off when on the ground. Returns final position/heading
  bool Takeoff(const std::string &ns, const double &takeoff_height, const double &sampling_time,
               const double &avg_velocity, ros::NodeHandle *nh, xyz_heading *final_xyz_yaw);

  // Method for landing from a current location
  bool Land(const std::string &ns, const double &land_height, const double &sampling_time,
            const double &avg_velocity, ros::NodeHandle *nh);

  // Method for landing from a current location (use px4_control autoland)
  bool Land(const std::string &ns, ros::NodeHandle *nh);

  // Method for going straight to a final point 
  bool GoStraight2Point(const std::string &ns, const xyz_heading &destination, const double &sampling_time,
                        const double &avg_velocity, ros::NodeHandle *nh);

  // Method for executing waypoint navigation  (blocks execution of code until finished)
  bool WaypointNavigation(const std::string &ns, const std::vector<xyz_heading> waypoints,
                          const Eigen::Vector3d &init_vel, const Eigen::Vector3d &final_vel,
                          const double &max_vel, const double &max_acc, const double &sampling_time,
                          ros::NodeHandle *nh);

  // Returns current pose of the vehicle - thread safe
  tf::StampedTransform GetCurrentPose();

  // Method for getting a minimum snap trajectory between two points only
  bool MinSnapPoint2Point(const std::string &ns, const Eigen::Vector3d &init_point,
                          const Eigen::Vector3d &final_point, const double &yaw0, 
                          const double &yaw_final, const double &tf, const double &sampling_time,
                          ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates);

  // Different template for the function above
  bool MinSnapPoint2Point(const std::string &ns, const xyz_heading &init_wp, 
                          const xyz_heading &final_wp, const double &tf, 
                          const double &sampling_time, ros::NodeHandle *nh,
                          mg_msgs::PVAJS_array *flatStates);

  // Method for getting minimum snap trajectories for multiple waypoints
  bool MinSnapWaypointSet(const std::string &ns, const std::vector<xyz_heading> waypoints,
                          const Eigen::Vector3d &init_vel, const Eigen::Vector3d &final_vel,
                          const double &max_vel, const double &max_acc, const double &sampling_time,
                          ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates);
  // Different template for the function above
  bool MinSnapWaypointSet(const std::string &ns, const minSnapWpInputs &Inputs,
                          ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates);

  // Method for sending a PVAJS trajectory to the action server
  bool CallPVAJSAction(const std::string &ns, const mg_msgs::PVAJS_array &flatStates,
                       const double &sampling_time, const bool &wait_until_done,
                       ros::NodeHandle *nh);

  // Similar to construction above, but the client is declared outside the function
  bool CallPVAJSAction(const std::string &ns, const mg_msgs::PVAJS_array &flatStates,
                       const double &sampling_time, const bool &wait_until_done,
                       ros::NodeHandle *nh,
                       actionlib::SimpleActionClient<mg_msgs::follow_PVAJS_trajectoryAction> *client);

  // Callbacks -----------------------------------------------------------------
  void ActionGoalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);


  // // Services (see services.cpp for implementation) -----------------
  // // Update resolution of the map
  // bool UpdateResolution(mapper::SetFloat::Request &req,
  //                       mapper::SetFloat::Response &res);

  // // Update map memory time
  // bool UpdateMemoryTime(mapper::SetFloat::Request &req,
  //                       mapper::SetFloat::Response &res);

  // // Update map inflation
  // bool MapInflation(mapper::SetFloat::Request &req,
  //                   mapper::SetFloat::Response &res);

  // // Reset the map
  // bool ResetMap(std_srvs::Trigger::Request &req,
  //               std_srvs::Trigger::Response &res);

  // // Threads (see threads.cpp for implementation) -----------------
  // // Thread for fading memory of the octomap
  // void FadeTask();

  // Threads for constantly updating the tfTree values
  void TfTask();

  // Thread that reads waypoints from a buffer to solve minimum snap problems
  void MinSnapSolverTask(const std::string &ns);

  // Consumer thread
  // Consumes minimum snap trajectories (sends to action server)
  void TrajectoryActionCaller(const std::string &ns);

  // void PerchTfTask();
  // void BodyTfTask();
  // void TfTask(const std::string& parent_frame,
  //             const std::string& child_frame,
  //             const uint& index); // Returns the transform from child to parent frame, expressed in parent frame

  // // Thread for collision checking
  // void CollisionCheckTask();

  // // Thread for getting pcl data and populating the octomap
  // void OctomappingTask();

  // // Thread for getting keyboard messages
  // void KeyboardTask();

 private:
  // Declare global variables (structures defined in structs.h)
  globalVariables globals_;  // These variables are all mutex-protected
  mutexStruct mutexes_;
  // semaphoreStruct semaphores_;

  // Namespace of the current node
  std::string ns_;

  // List of waypoints for inspection and localization procedures
  std::vector<xyz_heading> localization_waypoint_list_, inspection_waypoint_list_;

  // Thread variables
  std::thread h_tf_thread_, h_min_snap_thread_, h_trajectory_caller_thread_;

  // Subscriber variables
  ros::Subscriber action_status_sub_;

  // // Octomap services
  // ros::ServiceServer resolution_srv_, memory_time_srv_;
  // ros::ServiceServer map_inflation_srv_, reset_map_srv_;

  // Thread rates (hz)
  double tf_update_rate_;

  // Navigation parameters
  double avg_velocity_;

  // // Path planning services
  // ros::ServiceServer RRT_srv_, octoRRT_srv_, PRM_srv_, graph_srv_, Astar_srv_;
  // ros::ServiceServer newTraj_srv_;

  // // Marker publishers
  // ros::Publisher sentinel_pub_;
  // ros::Publisher obstacle_marker_pub_;
  // ros::Publisher free_space_marker_pub_;
  // ros::Publisher inflated_obstacle_marker_pub_;
  // ros::Publisher inflated_free_space_marker_pub_;
  // ros::Publisher path_marker_pub_;
  // ros::Publisher cam_frustum_pub_;
  // ros::Publisher map_keep_in_out_pub_;
};

}  // namespace mission_planner

#endif  // MISSION_CLASS_H_