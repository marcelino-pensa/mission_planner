

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

    // Takeoff
    ROS_INFO("[mission_node] Asking for takeoff command...");
    mission_.TakeoffMinTime(ns_, origin, takeoff_height-origin.z_, sampling_time,
             max_vel, max_acc, max_jerk, nh, &final_waypoint);
    
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

void InspectorClass::ShelfMission(ros::NodeHandle *nh) {
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

    // Takeoff position
    std::vector<mission_planner::xyz_heading> takeoff_list;
    takeoff_list.push_back(origin);
    takeoff_list.push_back(mission_planner::xyz_heading(0.0, 0.0, 0.65, 0.0));
    final_waypoint = takeoff_list.back();

    // Ingressing
    std::vector<mission_planner::xyz_heading> ingress_list;
    ingress_list.push_back(final_waypoint);
    ingress_list.push_back(mission_planner::xyz_heading(-0.3, -1.4, 0.65, 0.0));
    ingress_list.push_back(mission_planner::xyz_heading(-0.3, -1.4, -0.7, 0.0));
    final_waypoint = ingress_list.back();

    // Set mission waypoints
    std::vector<mission_planner::xyz_heading> scan_list;
    scan_list.push_back(final_waypoint);
    // scan_list.push_back(mission_planner::xyz_heading(1.4, -0.3, 0.65, 0.0));
    // scan_list.push_back(mission_planner::xyz_heading(1.4, -0.3, -0.7, 0.0));
    scan_list.push_back(mission_planner::xyz_heading(2.0, -1.4, -0.7, 0.0));
    scan_list.push_back(mission_planner::xyz_heading(4.0, -1.4, -0.7, 0.0));
    scan_list.push_back(mission_planner::xyz_heading(6.2, -1.4, -0.7, 0.0));
    scan_list.push_back(mission_planner::xyz_heading(6.2, -1.4, -1.4, 0.0));
    scan_list.push_back(mission_planner::xyz_heading(4.0, -1.4, -1.4, 0.0));
    scan_list.push_back(mission_planner::xyz_heading(2.0, -1.4, -1.4, 0.0));
    scan_list.push_back(mission_planner::xyz_heading(0.2, -1.4, -1.4, 0.0));
    scan_list.push_back(mission_planner::xyz_heading(0.2, -1.4, 0.65, 0.0));
    scan_list.push_back(mission_planner::xyz_heading(0.0, -0.0, 0.65, 0.0));
    final_waypoint = scan_list.back();

    // // Egress
    // std::vector<mission_planner::xyz_heading> egress_list;
    // egress_list.push_back(final_waypoint);
    // egress_list.push_back(mission_planner::xyz_heading(1.4, 0.2, 0.65, 0.0));
    // egress_list.push_back(mission_planner::xyz_heading(0.0, 0.0, 0.65, 0.0));
    // final_waypoint = egress_list.back();

    // Land
    std::vector<mission_planner::xyz_heading> land_list;
    land_list.push_back(final_waypoint);
    land_list.push_back(mission_planner::xyz_heading(0.0, 0.0, -0.5, 0.0));
    final_waypoint = land_list.back();

    // Go to takeoff position
    ROS_INFO("[mission_node] Asking for takeoff command...");
    mission_.AddMinTimeWp2Buffer(takeoff_list, max_vel, max_acc, max_jerk, sampling_time, &final_waypoint);

    // Ingress to scan area
    ROS_INFO("[mission_node] Ingress...");
    mission_.AddMinTimeWp2Buffer(ingress_list, max_vel, max_acc, max_jerk, sampling_time, &final_waypoint);
    
    // Add sets of waypoints to buffer
    ROS_INFO("[mission_node] Scan waypoints...");
    mission_.AddMinTimeWp2Buffer(scan_list, max_vel, max_acc, max_jerk, sampling_time, &final_waypoint);

    // Add sets of waypoints to buffer
    ROS_INFO("[mission_node] Scan waypoints...");
    mission_.AddMinTimeWp2Buffer(land_list, max_vel, max_acc, max_jerk, sampling_time, &final_waypoint);

    // // Add sets of waypoints to buffer
    // ROS_INFO("[mission_node] Scan waypoints...");
    // mission_.AddMinTimeWp2Buffer(egress_list, max_vel, max_acc, max_jerk, sampling_time, &final_waypoint);


    // // Disarm quad
    // this->DisarmQuad(ns_, nh);

    ros::Duration(5).sleep();

}

} // inspector