

#ifndef INSPECTOR_CLASS_H_
#define INSPECTOR_CLASS_H_

// Ros libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Mission Planner Class
#include <mission_planner/mission_class.h>
#include <mission_planner/xyz_heading.h>

// // Local defined libraries
// #include <mission_planner/drone_structs.h>
// #include <mission_planner/helper.h>
// #include <mission_planner/tf_class.h>

// // Msg/srv defined in other packages
// #include "mg_msgs/minSnapWpStamped.h"
// #include "mg_msgs/minSnapWpPVAJ.h"
// #include "mg_msgs/PVAJ_request.h"

// // Msg/srv types
// #include "std_srvs/Trigger.h"

// // C++ libraries
#include <vector>
#include <string>
#include <fstream>
// #include <math.h>
// #include <thread>

// Eigen-based libraries
#include <Eigen/Dense>

namespace inspector {

class InspectorClass {
 public:
  InspectorClass() {};
  ~InspectorClass() {};

  // Method for executing a mission. This is implemented in different files, and then
  // the compiled one is executed
  virtual void Mission(ros::NodeHandle *nh);

 protected:
  // Callbacks -------------------------------------------------------
  void LocalizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  // Method for loading waypoints from file
  bool LoadWaypoints(const std::string &filename,
                     const tf::StampedTransform &init_pose,
                     std::vector<mission_planner::xyz_heading> *waypoint_list);


 private:

  // Namespace of the current node
  std::string ns_;

  // List of waypoints for inspection and localization procedures
  std::vector<mission_planner::xyz_heading> localization_waypoint_list_, inspection_waypoint_list_;

  // Navigation parameters
  double max_velocity_;

  // Mission class
  mission_planner::MissionClass mission_;

  // Vector that contains corrections obtained from the localizing node
  std::vector<mission_planner::xyz_heading> correction_list_;

};

}  // namespace inspector

#endif  // INSPECTOR_CLASS_H_