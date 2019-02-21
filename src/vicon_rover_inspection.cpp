

#include <mission_planner/vicon_rover_inspection.h>

// ---------------------------------------------------
namespace inspector {
	
void InspectorClass::Mission(ros::NodeHandle *nh) {
    nh_ = *nh;

    // Get namespace of current node
    nh_.getParam("namespace", ns_);
    ROS_INFO("[mission_node] namespace: %s", ns_.c_str());

    // Get update rate for tf threads
    double tf_update_rate;
    nh_.getParam("tf_update_rate", tf_update_rate);

    // Get guidance parameters
    double takeoff_height;
    nh_.getParam("takeoff_height", takeoff_height);
    nh_.getParam("max_velocity", max_velocity_);

    // Get path for waypoint files
    std::string localization_file, inspection_file;
    nh_.getParam("localization_waypoints_path", localization_file);
    nh_.getParam("inspection_waypoints_path", inspection_file);

    // Start the Mission Planner Engine
    ROS_INFO("[mission_node] Starting Mission Planner Engine!");    
    mission_.Initialize(ns_, tf_update_rate, max_velocity_);

    // Start waypoint marker publisher and delete all markers currently published
    ROS_INFO("[mission_node] Creating visualization marker publisher!");    
    waypoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("MissionWps", 2, true);
    visualization_msgs::MarkerArray wp_markers;
    visualization_functions::DeleteMarkersTemplate(markers_frame_id_, &wp_markers);
    waypoint_marker_pub_.publish(wp_markers);

    // Wait until measurements are available
    ROS_INFO("[mission_node] Waiting for first pose in tf tree...");
    tf::StampedTransform tf_initial_pose = mission_.WaitForFirstPose();
    mission_planner::xyz_heading origin(tf_initial_pose);
    ROS_INFO("[mission_node] First pose obtained from tf tree!");

    // Variable for setting waypoints
    std::vector<mission_planner::xyz_heading> waypoints;
    mission_planner::xyz_heading final_waypoint;     // Variable used to save last waypoint on each sequence of waypoints

    // Navigation constant variables
    const double max_vel = max_velocity_, max_acc = 5.0;
    const Eigen::Vector3d init_vel(0.0, 0.0, 0.0), final_vel(0.0, 0.0, 0.0);
    const double sampling_time = 0.01;

    // Takeoff
    ROS_INFO("[mission_node] Taking off!");
    mission_.Takeoff(ns_, takeoff_height-origin.z_, sampling_time, max_velocity_, nh, &final_waypoint);

    // Wait until quad is done taking off before executing the next step of the mission
    ROS_INFO("[mission_node] Wait until quad is idle...");
    mission_.ReturnWhenIdle();
    ROS_INFO("[mission_node] Quad is idle!");

    // Start service for finding relative pose between SLAM and vicon frames
    std::string service_name = "/" + ns_ + "/batch_solver/start_new_batch";
    rel_pose_client_ = nh_.serviceClient<mg_msgs::RequestRelativePoseBatch>(service_name);
    mg_msgs::RequestRelativePoseBatch client_msg;
    client_msg.request.data = 50;
    ROS_INFO("Calling service '%s' for batch solution!", rel_pose_client_.getService().c_str());
    if (rel_pose_client_.call(client_msg)) {
        ROS_INFO("[mission_node] Relative pose returned successfully!");
    } else {
        ROS_WARN("[mission_node] Error calling relative pose server. Aborting!");
        return;
    }

    // Create thread that publishes relative tf between vicon and slam frames
    ROS_INFO("[mission_node] Publishing relative pose between vicon and slam frames!");
    geometry_msgs::Pose rel_pose = client_msg.response.pose;
    rel_tf_pub_thread_ = std::thread(&InspectorClass::RelTfPubTask, this, rel_pose);

    // Load inspection file, transforming data into vicon frame
    if(LoadWaypoints(inspection_file, rel_pose, &inspection_waypoint_list_) == 0) {
        return;
    } else {
        ROS_INFO("[mission_node] Inspection waypoints were loaded successfully. Number of waypoints: %d",
                 static_cast<int>(inspection_waypoint_list_.size()) );
    }

    // Publish inspection waypoint markers
    this->PublishWaypointMarkers(inspection_waypoint_list_);

    // Minimum snap cannot be solved for too many input waypoints (polynomial complexity). 
    // We divide the waypoints into subset of waypoints, which can be solved quickly.
    // In addition, a sequence of waypoints can be calculated while a trajectory is being executed.
    ROS_INFO("[mission_node] Splitting in segments...");
    std::vector<std::pair<uint, uint>> segments;
    uint wp_per_segment = 30;
    segments = helper::split_waypoints(inspection_waypoint_list_.size(), wp_per_segment);
    
    // Go to initial waypoint in the set
    ROS_INFO("[mission_node] Going to initial waypoint in the set...");
    waypoints.push_back(final_waypoint);
    waypoints.push_back(inspection_waypoint_list_[0]);
    mission_.AddWaypoints2Buffer(waypoints, init_vel, final_vel, max_vel, max_acc, sampling_time, &final_waypoint);
    
    // Add sets of waypoints to buffer
    ROS_INFO("[mission_node] Adding sets of waypoints to buffer...");
    for (uint k = 0; k < segments.size(); k++) {
        ROS_INFO("Adding waypoints %d to %d into buffer.", int(segments[k].first), int(segments[k].second));
        waypoints.clear();
        for (uint i = segments[k].first; i <= segments[k].second; i++) {
            waypoints.push_back(inspection_waypoint_list_[i]);
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

    // Wait until quad is done with landing before executing the next step of the mission
    ROS_INFO("[mission_node] Wait until quad is idle...");
    mission_.ReturnWhenIdle();
    ROS_INFO("[mission_node] Quad is idle!");

    // Disarm quad
    mission_.DisarmQuad(ns_, nh);

}

bool InspectorClass::LoadWaypoints(const std::string &filename,
                                   const geometry_msgs::Pose &rel_pose,
                                   std::vector<mission_planner::xyz_heading> *waypoint_list) {
    ROS_INFO("[mission_node] Opening waypoints file: \n%s\n", filename.c_str());
    std::ifstream myfile(filename.c_str());
    float x, y, z, yaw;
    Eigen::Vector3d rel_pos = msg_conversions::ros_point_to_eigen_vector(rel_pose.position);
    Eigen::Quaterniond rel_att = msg_conversions::ros_to_eigen_quat(rel_pose.orientation);
    Eigen::Matrix3d rot = rel_att.toRotationMatrix();
    Eigen::Vector3d rpy = helper::quat2rpy(rel_pose.orientation);
    float rel_yaw = rpy(2);
    // init_pose.getBasis().getRPY(init_roll, init_pitch, init_yaw);
    // ROS_INFO("Init yaw: %f", init_yaw);


    // Check whether file could be opened (path might be wrong)
    if (myfile.is_open()) {
        while( myfile >> x >> y >> z >> yaw) {
            Eigen::Vector3d pos_slam(x, y, z);
            Eigen::Vector3d pos_vicon = rel_pos + rot*pos_slam;
            waypoint_list->push_back(mission_planner::xyz_heading(pos_vicon, rel_yaw + yaw));
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

void InspectorClass::PublishWaypointMarkers(const std::vector<mission_planner::xyz_heading> &waypoint_list) {
    // Set marker properties
    double diameter = 0.01;
    std_msgs::ColorRGBA color = visualization_functions::Color::Cyan();
    std::string ns = "waypoints";
    double marker_length = 0.1;
    visualization_msgs::MarkerArray wp_markers;

    for (uint i = 0; i < waypoint_list.size(); i++) {
        // Get initial and final points
        double yaw = waypoint_list[i].GetYaw();
        Eigen::Vector3d p1 = waypoint_list[i].GetEigenXYZ();
        Eigen::Vector3d p2 = p1 + marker_length*Eigen::Vector3d(cos(yaw), sin(yaw), 0.0);
        
        // Get visualization marker for the waypoint
        visualization_msgs::Marker waypoint;
        visualization_functions::DrawArrowPoints(p1, p2, color, markers_frame_id_, ns, i, 
                                                 diameter, &waypoint);
        wp_markers.markers.push_back(waypoint);
    }

    // Publish all waypoints
    waypoint_marker_pub_.publish(wp_markers);
}

void InspectorClass::RelTfPubTask(const geometry_msgs::Pose &pose) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  transform.setRotation(q);
  
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "slam"));
    loop_rate.sleep();
  }
}

} // inspector