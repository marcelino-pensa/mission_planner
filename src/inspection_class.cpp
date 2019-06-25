

#include <mission_planner/inspection_class.h>

// ---------------------------------------------------
namespace inspector {
	
void InspectorClass::Mission(ros::NodeHandle *nh) {
    // Get namespace of current node
    nh->getParam("namespace", ns_);
    ROS_INFO("[mission_node] namespace: %s", ns_.c_str());

    // Get update rate for tf threads
    double tf_update_rate;
    nh->getParam("tf_update_rate", tf_update_rate);

    // Get guidance parameters
    double takeoff_height;
    nh->getParam("takeoff_height", takeoff_height);
    nh->getParam("max_velocity", max_velocity_);
    nh->getParam("max_acceleration", max_acceleration_);
    nh->getParam("max_jerk", max_jerk_);

    // Start the Mission Planner Engine
    ROS_INFO("[mission_node] Initialize mission...");
    mission_.Initialize(ns_, tf_update_rate, max_velocity_, max_acceleration_, max_jerk_);

    // Wait until measurements are available
    ROS_INFO("[mission_node] Wait for first pose...");
    tf::StampedTransform tf_initial_pose = mission_.WaitForFirstPose();
    mission_planner::xyz_heading origin(tf_initial_pose);
    ROS_INFO("[mission_node] First pose obtained from tf tree!");
    // mission_planner::xyz_heading origin(0.0, 0.0, 0.0, 0.0);

    // Variable for setting waypoints
    std::vector<mission_planner::xyz_heading> waypoints;
    mission_planner::xyz_heading final_waypoint;     // Variable used to save last waypoint on each sequence of waypoints

    // Navigation constant variables
    const double max_vel = max_velocity_, max_acc = max_acceleration_, max_jerk = max_jerk_;
    const double sampling_time = 0.025;

    // Set waypoints
    std::vector<mission_planner::xyz_heading> waypoint_list;
    waypoint_list.push_back(mission_planner::xyz_heading(0.0, 0.0, 0.75, 0.0));
    waypoint_list.push_back(mission_planner::xyz_heading(1.0, 0.0, 0.75, 0.0));
    waypoint_list.push_back(mission_planner::xyz_heading(2.0, 0.0, 0.75, 0.0));
    waypoint_list.push_back(mission_planner::xyz_heading(3.0, 0.0, 0.75, 0.0));
    waypoint_list.push_back(mission_planner::xyz_heading(4.0, 0.0, 0.75, 0.0));
    waypoint_list.push_back(mission_planner::xyz_heading(4.0, 0.0, 1.25, 0.0));
    waypoint_list.push_back(mission_planner::xyz_heading(3.0, 0.0, 1.25, 0.0));
    waypoint_list.push_back(mission_planner::xyz_heading(2.0, 0.0, 1.25, 0.0));
    waypoint_list.push_back(mission_planner::xyz_heading(1.0, 0.0, 1.25, 0.0));
    waypoint_list.push_back(mission_planner::xyz_heading(0.0, 0.0, 1.25, 0.0));
    // waypoint_list.push_back(mission_planner::xyz_heading(0.0, 0.0, 2.0, 0.0));
    // waypoint_list.push_back(mission_planner::xyz_heading(1.0, 0.0, 2.0, 0.0));
    // waypoint_list.push_back(mission_planner::xyz_heading(2.0, 0.0, 2.0, 0.0));
    // waypoint_list.push_back(mission_planner::xyz_heading(3.0, 0.0, 2.0, 0.0));
    // waypoint_list.push_back(mission_planner::xyz_heading(4.0, 0.0, 2.0, 0.0));

    // Takeoff
    ROS_INFO("[mission_node] Asking for takeoff command...");
    mission_.TakeoffMinTime(ns_, origin, takeoff_height-origin.z_, sampling_time,
             max_vel, max_acc, max_jerk, nh, &final_waypoint);
    
    // // Go to initial waypoint in the set
    // ROS_INFO("[mission_node] Go to initial waypoint...");
    // waypoints.push_back(final_waypoint);
    // waypoints.push_back(waypoint_list[0]);
    // mission_.AddMinTimeWp2Buffer(waypoints, max_vel, max_acc, max_jerk, sampling_time, &final_waypoint);
    
    // Add sets of waypoints to buffer
    ROS_INFO("[mission_node] Plan waypoints...");
    mission_.AddMinTimeWp2Buffer(waypoint_list, max_vel, max_acc, max_jerk, sampling_time, &final_waypoint);

    // // Go to origin
    // ROS_INFO("[mission_node] Go back to origin...");
    // waypoints.clear();
    // waypoints.push_back(final_waypoint);
    // waypoints.push_back(mission_planner::xyz_heading(origin.x_, origin.y_, 1.0, origin.yaw_));
    // mission_.AddMinTimeWp2Buffer(waypoints, max_vel, max_acc, max_jerk, sampling_time, &final_waypoint);

    // Land
    ROS_INFO("[mission_node] Land...");
    waypoints.clear();
    waypoints.push_back(final_waypoint);
    waypoints.push_back(mission_planner::xyz_heading(final_waypoint.x_, final_waypoint.y_, 0.0, final_waypoint.yaw_));
    mission_.AddMinTimeWp2Buffer(waypoints, max_vel, max_acc, max_jerk, sampling_time, &final_waypoint);
    
    // // Wait until quad is done with landing before executing the next step of the mission
    // ROS_INFO("[mission_node] Wait until quad is idle...");
    // mission_.ReturnWhenIdle();
    // ROS_INFO("[mission_node] Quad is idle!");


    // // Disarm quad
    // this->DisarmQuad(ns_, nh);

    ros::Duration(5).sleep();

}

} // inspector