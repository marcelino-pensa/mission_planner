
#include <mission_planner/mission_class.h>


// ---------------------------------------------------
namespace mission_planner {
	
void MissionClass::Mission(ros::NodeHandle *nh) {
    // Get namespace of current node
    nh->getParam("namespace", ns_);
    ROS_INFO("[mission_node] namespace: %s", ns_.c_str());

    // Get update rate for tf threads
    nh->getParam("tf_update_rate", tf_update_rate_);

    // Get guidance parameters
    double takeoff_height;
    nh->getParam("takeoff_height", takeoff_height);
    nh->getParam("avg_velocity", avg_velocity_);

    // Get path for waypoint files
    std::string localization_file, inspection_file;
    nh->getParam("localization_waypoints_path", localization_file);
    nh->getParam("inspection_waypoints_path", inspection_file);

    // Start subscribers ----------------------------------------------------------
    std::string topic_name = "/" + ns_ + "/follow_PVAJS_trajectory_action/status";
    action_status_sub_ = nh->subscribe(topic_name, 10, &MissionClass::ActionGoalStatusCallback, this);

    // Start threads --------------------------------------------------------------
    h_tf_thread_ = std::thread(&MissionClass::TfTask, this);
    h_min_snap_thread_ = std::thread(&MissionClass::MinSnapSolverTask, this, ns_);
    h_trajectory_caller_thread_ = std::thread(&MissionClass::TrajectoryActionCaller, this, ns_);

    // Wait until measurements are available
    tf::StampedTransform tf_initial_pose;
    ros::Rate loop_rate(10);
    do {
        loop_rate.sleep();
        mutexes_.tf.lock();
            tf_initial_pose = globals_.tf_quad2world;
        mutexes_.tf.unlock();
    } while (tf_initial_pose.stamp_.toSec() <= 0.0);

    // Load localization file
    if(!LoadWaypoints(inspection_file, tf_initial_pose, &inspection_waypoint_list_)) {
        return;
    } else {
        ROS_INFO("[mission_node] Inspection waypoints were loaded successfully. Number of waypoints: %d",
                 static_cast<int>(inspection_waypoint_list_.size()) );
    }

    // // Publish inspection points in Rviz
    // visualization_msgs::MarkerArray marker_array;
    // drawWaypoints(inspection_waypoint_list_, "map", &marker_array);
    // ros::Publisher wpMarker_pub = nh->advertise<visualization_msgs::MarkerArray>
    //                                 ("waypoint_visualization_markers", 1000);
    // ros::Rate loop_rate2(1);
    // for(uint i = 0; i < 20; i++) {
    //     wpMarker_pub.publish(marker_array);
    //     loop_rate2.sleep();
    // }
    


    // Load inspection file
    if(!LoadWaypoints(localization_file, tf_initial_pose, &localization_waypoint_list_)) {
        return;
    } else {
        ROS_INFO("[mission_node] Localization waypoints were loaded successfully. Number of waypoints: %d",
                 static_cast<int>(localization_waypoint_list_.size()) );
    }

    // Minimum snap cannot be solved for too many input waypoints (polynomial complexity). 
    // We divide the waypoints into subset of waypoints, which can be solved quickly.
    // In addition, a sequence of waypoints can be calculated while a trajectory is being executed.
    std::vector<std::pair<uint, uint>> segments;
    uint wp_per_segment = 50;
    segments = helper::split_waypoints(inspection_waypoint_list_.size(), wp_per_segment);

    // Variable for setting waypoints
    std::vector<xyz_heading> waypoints;
    xyz_heading final_waypoint;     // Variable used to save last waypoint on each sequence of waypoints

    // Navigation constant variables
    const double max_vel = avg_velocity_, max_acc = 10.0;
    const Eigen::Vector3d init_vel(0.0, 0.0, 0.0), final_vel(0.0, 0.0, 0.0);
    const double sampling_time = 0.01;

    // Takeoff
    this->Takeoff(ns_, takeoff_height, sampling_time, avg_velocity_, nh, &final_waypoint);
    
    // Go to initial waypoint in the set
    waypoints.push_back(final_waypoint);
    waypoints.push_back(inspection_waypoint_list_[0]);
    this->AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);
    
    // Add sets of waypoints to buffer
    for (uint k = 0; k < segments.size(); k++) {
        ROS_INFO("Adding waypoints %d to %d into buffer.", int(segments[k].first), int(segments[k].second));
        waypoints.clear();
        for (uint i = segments[k].first; i <= segments[k].second; i++) {
            waypoints.push_back(inspection_waypoint_list_[i]);
        }
        this->AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);
    }

    ros::Duration(200.0).sleep();

    // Go to origin
    waypoints.clear();
    waypoints.push_back(final_waypoint);
    waypoints.push_back(xyz_heading(0.0, 0.0, final_waypoint.z_, 0.0));
    this->AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);

    // Land
    waypoints.clear();
    waypoints.push_back(final_waypoint);
    waypoints.push_back(xyz_heading(final_waypoint.x_, final_waypoint.y_, 0.0, final_waypoint.yaw_));
    this->AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);

//     // // Run through a few waypoints
//     // double min_dist = 1.0;
//     // tf::StampedTransform initial_pose = this->GetCurrentPose();
//     // waypoints.push_back(mission_planner::xyz_heading(initial_pose));  // Current pose as initial waypoint

//     // Eigen::Vector3d vec_a2b = inspection_waypoint_list_[0].GetEigenXYZ() - waypoints[0].GetEigenXYZ();
//     // double yaw_a2b = inspection_waypoint_list_[0].GetYaw() - waypoints[0].GetYaw();
//     // double distance = vec_a2b.norm();
//     // if (distance > min_dist) {
//     //     Eigen::Vector3d direction = vec_a2b.normalized();
//     //     double distance_ratio = distance/min_dist;
//     //     double n_new_samples = std::floor(distance_ratio);
//     //     double step = distance_ratio/(n_new_samples + 1.0);
//     //     double yaw_step = yaw_a2b/(n_new_samples + 1.0);
//     //     for (uint i = 0; i < n_new_samples; i++) {
//     //         Eigen::Vector3d new_xyz = waypoints[0].GetEigenXYZ() + double(i+1)*step*direction;
//     //         double new_yaw = waypoints[0].GetYaw() + double(i+1)*yaw_step;
//     //         waypoints.push_back(mission_planner::xyz_heading(new_xyz, new_yaw));
//     //     }
//     // }
    
    // // Disarm quad
    // this->DisarmQuad(ns_, nh);

}

} // mission_planner