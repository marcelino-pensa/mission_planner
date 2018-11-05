

#include <mission_planner/inspection_class.h>

// ---------------------------------------------------
namespace inspector {

void InspectorClass::LocalizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // Get SLAM pose and measured pose
    geometry_msgs::PoseWithCovarianceStamped local_pose = *msg;
    double local_yaw = helper::getHeadingFromQuat(local_pose.pose.pose.orientation);
    tf::StampedTransform current_pose = mission_.GetCurrentPose();
    double current_yaw = helper::getHeadingFromTransform(current_pose);

    mission_planner::xyz_heading potential_correction_pose;
    potential_correction_pose.x_ = current_pose.getOrigin().x() - local_pose.pose.pose.position.x;
    potential_correction_pose.y_ = current_pose.getOrigin().y() - local_pose.pose.pose.position.y;
    potential_correction_pose.z_ = current_pose.getOrigin().z() - local_pose.pose.pose.position.z;

    potential_correction_pose.yaw_ = current_yaw - local_yaw;
    correction_list_.push_back(potential_correction_pose);
    ROS_INFO("Current heading %f\t Map heading: %f", current_yaw, local_yaw);
    ROS_INFO("potential correction heading %f", potential_correction_pose.yaw_);
    ROS_INFO("Correction Vector x: %4.3f\ty: %4.3f\tz: %4.3f " , potential_correction_pose.x_,
             potential_correction_pose.y_, potential_correction_pose.z_);
    //ROS_INFO("LOCAL POSITION RECIEVED %f %f %f", local_pose.pose.position.x, local_pose.pose.position.y, localPose.pose.position.z);
}
	
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

    // Get path for waypoint files
    std::string localization_file, inspection_file;
    nh->getParam("localization_waypoints_path", localization_file);
    nh->getParam("inspection_waypoints_path", inspection_file);

    // Start the Mission Planner Engine
    mission_.Initialize(ns_, tf_update_rate, max_velocity_);

    // Start subscribers ----------------------------------------------------------
    ros::Subscriber localization = nh->subscribe
                ("/rtabmap/localization_pose", 1, &InspectorClass::LocalizationCallback,this);
    // std::string topic_name = "/" + ns_ + "/follow_PVAJS_trajectory_action/status";
    // action_status_sub_ = nh->subscribe(topic_name, 10, &MissionClass::ActionGoalStatusCallback, this);

    // Wait until measurements are available
    tf::StampedTransform tf_initial_pose = mission_.WaitForFirstPose();
    mission_planner::xyz_heading origin(tf_initial_pose);

    // Load localization file
    if(!LoadWaypoints(inspection_file, tf_initial_pose, &inspection_waypoint_list_)) {
        return;
    } else {
        ROS_INFO("[mission_node] Inspection waypoints were loaded successfully. Number of waypoints: %d",
                 static_cast<int>(inspection_waypoint_list_.size()) );
    }

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
    uint wp_per_segment = 20;
    segments = helper::split_waypoints(localization_waypoint_list_.size(), wp_per_segment);

    // Variable for setting waypoints
    std::vector<mission_planner::xyz_heading> waypoints;
    mission_planner::xyz_heading final_waypoint;     // Variable used to save last waypoint on each sequence of waypoints

    // Navigation constant variables
    const double max_vel = max_velocity_, max_acc = 10.0;
    const Eigen::Vector3d init_vel(0.0, 0.0, 0.0), final_vel(0.0, 0.0, 0.0);
    const double sampling_time = 0.01;

    // Takeoff
    mission_.Takeoff(ns_, takeoff_height, sampling_time, max_velocity_, nh, &final_waypoint);
    
    // Go to initial waypoint in the set
    waypoints.push_back(final_waypoint);
    waypoints.push_back(localization_waypoint_list_[0]);
    mission_.AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);
    
    // Add sets of waypoints to buffer
    for (uint k = 0; k < segments.size(); k++) {
        ROS_INFO("Adding waypoints %d to %d into buffer.", int(segments[k].first), int(segments[k].second));
        waypoints.clear();
        for (uint i = segments[k].first; i <= segments[k].second; i++) {
            waypoints.push_back(localization_waypoint_list_[i]);
        }
        mission_.AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);
    }

    // Go to origin
    waypoints.clear();
    waypoints.push_back(final_waypoint);
    waypoints.push_back(mission_planner::xyz_heading(origin.x_, origin.y_, final_waypoint.z_, origin.yaw_));
    mission_.AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);

    // Land
    waypoints.clear();
    waypoints.push_back(final_waypoint);
    waypoints.push_back(mission_planner::xyz_heading(final_waypoint.x_, final_waypoint.y_, 0.0, final_waypoint.yaw_));
    mission_.AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);
    
    // // Disarm quad
    // this->DisarmQuad(ns_, nh);

}

bool InspectorClass::LoadWaypoints(const std::string &filename,
                                   const tf::StampedTransform &init_pose,
                                   std::vector<mission_planner::xyz_heading> *waypoint_list) {
    ROS_INFO("[mission_node] Opening waypoints file: \n%s\n", filename.c_str());
    std::ifstream myfile(filename.c_str());
    float x, y, z, yaw;
    double init_roll, init_pitch, init_yaw;
    init_pose.getBasis().getRPY(init_roll, init_pitch, init_yaw);
    // ROS_INFO("Init yaw: %f", init_yaw);


    // Check whether file could be opened (path might be wrong)
    if (myfile.is_open()) {
        // Observation: for some reason Eric defined yaw using a NED frame, while xyz in ENU frame
        // Observation2: waypoints are in relation with initial position
        while( myfile >> x >> y >> z >> yaw) {
            waypoint_list->push_back(mission_planner::xyz_heading(x, y, z, init_yaw+helper::deg2rad(yaw)));
            // std::cout << x << " " << y << " " << z << " " << init_yaw-yaw << std::endl;
        }
        myfile.close();

        // Check if any waypoint was loaded
        if(waypoint_list->size() > 0) {
            return 1;
        } else {
            ROS_ERROR("[mission_node] No waypoints within the file: %s", filename.c_str());
            return 0;
        }
    } else {
        ROS_ERROR("[mission_node] Unable to open file in %s", filename.c_str());
        return 0;
    }
}

} // inspector