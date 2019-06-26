

#ifndef INSPECTOR_CLASS_H_
#define INSPECTOR_CLASS_H_

// Ros libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Mission Planner Class
#include <mission_planner/mission_class.h>
#include <mission_planner/xyz_heading.h>

// Local defined libraries
#include "p4_ros/min_time.h"
// #include <mission_planner/drone_structs.h>
// #include <mission_planner/helper.h>
// #include <mission_planner/tf_class.h>

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

  virtual void ShelfMission(ros::NodeHandle *nh);

 private:

  // Namespace of the current node
  std::string ns_;

  // Navigation parameters
  double max_velocity_, max_acceleration_, max_jerk_;

  // Mission class
  mission_planner::MissionClass mission_;

};

}  // namespace inspector

#endif  // INSPECTOR_CLASS_H_