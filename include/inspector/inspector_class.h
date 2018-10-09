

#ifndef MAPPER_MAPPER_CLASS_H_
#define MAPPER_MAPPER_CLASS_H_

#include <ros/ros.h>

// Local defined libraries
#include <inspector/structs.h>
#include <inspector/helper.h>

// Msg/srv types
#include "std_srvs/Trigger.h"

// C++ libraries
#include <vector>
#include <string>
#include <thread>
#include <fstream>

namespace inspector {

class InspectorClass {
 public:
  InspectorClass();
  ~InspectorClass();

  // Constructor
  virtual void Initialize(ros::NodeHandle *nh);

 protected:
  // Callbacks (see callbacks.cpp for implementation) ----------------
  // Callback for handling incoming point cloud messages
  // void PclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg,
  //                  const uint& cam_index);

  // Method for loading waypoints from file
  bool LoadWaypoints(std::string &filename, 
                     std::vector<xyz_heading> *waypoint_list);

  // Method for starting the quadcopter to listen to PVA messages
  void SetQuadPosMode(std::string &ns, ros::NodeHandle *nh);

  // // Callback for handling incoming new trajectory messages
  // void SegmentCallback(const mapper::Segment::ConstPtr &msg);


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

  // // Threads for constantly updating the tfTree values
  // void HazTfTask();
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
  // globalVariables globals_;  // These variables are all mutex-protected
  // mutexStruct mutexes_;
  // semaphoreStruct semaphores_;

  // List of waypoints
  std::vector<xyz_heading> localization_waypoint_list_, inspection_waypoint_list_;

  // Thread variables
  std::thread h_haz_tf_thread_, h_perch_tf_thread_, h_body_tf_thread_;
  std::thread h_octo_thread_, h_fade_thread_, h_collision_check_thread_;
  std::thread h_keyboard_thread_;
  std::vector<std::thread> h_cameras_tf_thread_;

  // Subscriber variables
  ros::Subscriber haz_sub_, perch_sub_, segment_sub_;
  std::vector<ros::Subscriber> cameras_sub_;

  // Octomap services
  ros::ServiceServer resolution_srv_, memory_time_srv_;
  ros::ServiceServer map_inflation_srv_, reset_map_srv_;

  // Thread rates (hz)
  double tf_update_rate_, fading_memory_update_rate_;

  // Path planning services
  ros::ServiceServer RRT_srv_, octoRRT_srv_, PRM_srv_, graph_srv_, Astar_srv_;
  ros::ServiceServer newTraj_srv_;

  // Marker publishers
  ros::Publisher sentinel_pub_;
  ros::Publisher obstacle_marker_pub_;
  ros::Publisher free_space_marker_pub_;
  ros::Publisher inflated_obstacle_marker_pub_;
  ros::Publisher inflated_free_space_marker_pub_;
  ros::Publisher path_marker_pub_;
  ros::Publisher cam_frustum_pub_;
  ros::Publisher map_keep_in_out_pub_;
};

}  // namespace inspector

#endif  // MAPPER_MAPPER_CLASS_H_