

// Standard includes
#include <inspector/inspector_class.h>

// ---------------------------------------------------
namespace inspector {

InspectorClass::InspectorClass() {

}

InspectorClass::~InspectorClass() {
    // Join all threads
    h_tf_thread_.join();
    // h_perch_tf_thread_.join();
    // h_body_tf_thread_.join();
    // h_octo_thread_.join();
    // h_fade_thread_.join();
    // h_collision_check_thread_.join();

    // for(uint i = 0; i < h_cameras_tf_thread_.size(); i++) {
    //   h_cameras_tf_thread_[i].join();
    // }

    // // destroy mutexes and semaphores
    // mutexes_.destroy();
    // semaphores_.destroy();
}

// Thread for constantly updating the tfTree values
void InspectorClass::TfTask() {
    ROS_DEBUG("[inspector_node] tf Thread started with rate %f: ", tf_update_rate_);
    tf_listener::TfClass obj_quad2world;
    ros::Rate loop_rate(tf_update_rate_);

    std::string vehicle_frame = ns_ + "/base_link";
    std::string map_frame = "map";

    while (ros::ok()) {
        // Get the transforms
        obj_quad2world.GetTransform(vehicle_frame , map_frame);
        // obj_haz2body.getTransform("/haz_cam", "/body");

        mutexes_.tf.lock();
            globals_.tf_quad2world = obj_quad2world.transform_;
        mutexes_.tf.unlock();

        // obj_haz2body.printTransform();
        loop_rate.sleep();
    }

    ROS_DEBUG("[inspector_node] Exiting tf Thread...");
}

void InspectorClass::Initialize(ros::NodeHandle *nh) {
    // Get namespace of current node
    nh->getParam("namespace", ns_);
    ROS_INFO("[inspector_node] namespace: %s", ns_.c_str());

    // Get update rate for tf threads
    nh->getParam("tf_update_rate", tf_update_rate_);

    // Get guidance parameters
    double takeoff_height, avg_velocity;
    nh->getParam("takeoff_height", takeoff_height);
    nh->getParam("avg_velocity", avg_velocity);

    // Get path for waypoint files
    std::string localization_file, inspection_file;
    nh->getParam("localization_waypoints_path", localization_file);
    nh->getParam("inspection_waypoints_path", inspection_file);

    // Load localization file
    if(!LoadWaypoints(localization_file, &inspection_waypoint_list_)) {
        return;
    } else {
        ROS_INFO("[inspector_node] Localization waypoints were loaded successfully. Number of waypoints: %d",
                 static_cast<int>(inspection_waypoint_list_.size()) );
    }

    // Load inspection file
    if(!LoadWaypoints(inspection_file, &localization_waypoint_list_)) {
        return;
    } else {
        ROS_INFO("[inspector_node] Inspection waypoints were loaded successfully. Number of waypoints: %d",
                 static_cast<int>(localization_waypoint_list_.size()) );
    }

    // Start thread that gets pose measurements of the vehicle
    h_tf_thread_ = std::thread(&InspectorClass::TfTask, this);

    // Wait until measurements are available
    tf_listener::TfClass tf_initial_pose;
    ros::Rate loop_rate(10);
    do {
        loop_rate.sleep();
        mutexes_.tf.lock();
            tf_initial_pose.transform_ = globals_.tf_quad2world;
        mutexes_.tf.unlock();
    } while (tf_initial_pose.transform_.stamp_.toSec() <= 0.0);

    // Takeoff and land
    double sampling_time = 0.01;
    double land_height = 0.0;
    this->Takeoff(ns_, takeoff_height, sampling_time, avg_velocity, nh);
    this->Land(ns_, land_height, sampling_time, avg_velocity, nh);

    // Disarm quad
    this->DisarmQuad(ns_, nh);

}

bool InspectorClass::LoadWaypoints(const std::string &filename,
                                   std::vector<xyz_heading> *waypoint_list) {
    ROS_INFO("[inspector_node] Opening waypoints file: \n%s\n", filename.c_str());
    std::ifstream myfile(filename.c_str());
    float x, y, z, yaw;

    // Check whether file could be opened (path might be wrong)
    if (myfile.is_open()) {
        while( myfile >> x >> y >> z >> yaw) {
            waypoint_list->push_back(xyz_heading(x, y, z, helper::deg2rad(yaw)));
            // std::cout << x << " " << y << " " << z << " " << yaw << std::endl;
        }
        myfile.close();

        // Check if any waypoint was loaded
        if(waypoint_list->size() > 0) {
            return 1;
        } else {
            ROS_ERROR("[inspector_node] No waypoints within the file: %s", filename.c_str());
            return 0;
        }
    } else {
        ROS_ERROR("[inspector_node] Unable to open file in %s", filename.c_str());
        return 0;
    }
}

void InspectorClass::SetQuadPosMode(const std::string &ns, ros::NodeHandle *nh) {
    ROS_INFO("[inspector_node] Requesting Vehicle to Start Position Mode! This requires px4_control_node to be executing!");

    std::string service_name = "/" + ns + "/px4_control_node/setQuadPVAMode";
    ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>(service_name);
    
    if(!client.waitForExistence(ros::Duration(1.0))) {
        ROS_ERROR("[inspector_node] Service ""%s"" unavailable for call.", client.getService().c_str());
    }

    std_srvs::Trigger trigger_msg;
    if (client.call(trigger_msg)) {
        if(trigger_msg.response.success == false) {
            ROS_ERROR("[inspector_node] Failed to set vehicle to listen to position commands.");
        }
    } else {
        ROS_ERROR("[inspector_node] Failed to call service.");
    }
}

void InspectorClass::DisarmQuad(const std::string &ns, ros::NodeHandle *nh) {
    ROS_INFO("[inspector_node] Requesting Vehicle to Disarm Motors! This requires px4_control_node to be executing!");

    std::string service_name = "/" + ns + "/px4_control_node/disarmQuad";
    ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>(service_name);
    
    if(!client.waitForExistence(ros::Duration(1.0))) {
        ROS_ERROR("[inspector_node] Service ""%s"" unavailable for call.", client.getService().c_str());
    }

    std_srvs::Trigger trigger_msg;
    if (client.call(trigger_msg)) {
        if(trigger_msg.response.success == false) {
            ROS_ERROR("[inspector_node] Failed to set vehicle to listen to disarm.");
        }
    } else {
        ROS_ERROR("[inspector_node] Failed to call service.");
    }
}

// Method for taking off when on the ground 
bool InspectorClass::Takeoff(const std::string &ns, const double &takeoff_height, const double &sampling_time,
                             const double &avg_velocity, ros::NodeHandle *nh) {
    // Get current pose
    tf_listener::TfClass tf_initial_pose;
    mutexes_.tf.lock();
        tf_initial_pose.transform_ = globals_.tf_quad2world;
    mutexes_.tf.unlock();
    double yaw, pitch, roll;
    tf_initial_pose.transform_.getBasis().getRPY(roll, pitch, yaw);

    // Plan a minimum snap trajectory for taking-off
    double tf = takeoff_height/avg_velocity;
    Eigen::Vector3d init_pos = helper::rostfvec2eigenvec(tf_initial_pose.transform_.getOrigin());
    Eigen::Vector3d pos_final = init_pos + Eigen::Vector3d(0.0, 0.0, takeoff_height);

    // Get smooth trajectory for take-off
    mg_msgs::PVAJS_array flatStates;
    if (!this->MinSnapPoint2Point(ns, init_pos, pos_final, yaw, yaw, 
                                  tf, sampling_time, nh, &flatStates)) {
        return 0;
    }

    bool wait_until_done = true;
    if (!this->CallPVAJSAction(ns, flatStates, sampling_time, wait_until_done, nh)) {
        return 0;
    }
}

// Method for landing from a current location
bool InspectorClass::Land(const std::string &ns, const double &land_height, const double &sampling_time,
                          const double &avg_velocity, ros::NodeHandle *nh) {
    // Get current pose
    tf_listener::TfClass tf_initial_pose;
    mutexes_.tf.lock();
        tf_initial_pose.transform_ = globals_.tf_quad2world;
    mutexes_.tf.unlock();
    double yaw, pitch, roll;
    tf_initial_pose.transform_.getBasis().getRPY(roll, pitch, yaw);

    // Plan a minimum snap trajectory for taking-off
    Eigen::Vector3d init_pos = helper::rostfvec2eigenvec(tf_initial_pose.transform_.getOrigin());
    Eigen::Vector3d pos_final = init_pos;
    pos_final[2] = land_height;
    double height = fabs(init_pos[2] - land_height);
    double tf = height/avg_velocity;

    // Get smooth trajectory for landing
    mg_msgs::PVAJS_array flatStates;
    if (!this->MinSnapPoint2Point(ns, init_pos, pos_final, yaw, yaw, 
                                  tf, sampling_time, nh, &flatStates)) {
        return 0;
    }

    bool wait_until_done = true;
    if (!this->CallPVAJSAction(ns, flatStates, sampling_time, wait_until_done, nh)) {
        return 0;
    }

}

bool InspectorClass::MinSnapPoint2Point(const std::string &ns, const Eigen::Vector3d &init_point, 
                                        const Eigen::Vector3d &final_point, const double &yaw0,
                                        const double &yaw_final, const double &tf, const double &sampling_time,
                                        ros::NodeHandle *nh, mg_msgs::PVAJS_array *flatStates) {
    // Set service client
    std::string service_name = "/" + ns + "/minSnap";
    ros::ServiceClient client = nh->serviceClient<mg_msgs::minSnapStamped>(service_name);
    geometry_msgs::PoseStamped Pos0, Pos_mid, Pos_final;

    // Set first point
    Pos0.pose.position = helper::eigenvec2rospoint(init_point);
    Pos0.pose.orientation = helper::set_quat(0.0, 0.0, yaw0);
    Pos0.header.stamp = ros::Time(0.0);

    // An intermediate point is needed for the minimum snap solver
    Pos_mid.pose.position = helper::eigenvec2rospoint(0.5*(init_point + final_point));
    Pos_mid.pose.orientation = helper::set_quat(0.0, 0.0, 0.5*(yaw0 + yaw_final));
    Pos_mid.header.stamp = ros::Time(0.5*tf);

    // Set final point
    Pos_final.pose.position = helper::eigenvec2rospoint(final_point);
    Pos_final.pose.orientation = helper::set_quat(0.0, 0.0, yaw_final);
    Pos_final.header.stamp = ros::Time(tf);

    // Push points into waypoint structure
    nav_msgs::Path Waypoints;
    Waypoints.poses.push_back(Pos0);
    Waypoints.poses.push_back(Pos_mid);
    Waypoints.poses.push_back(Pos_final);

    mg_msgs::minSnapStamped srv;
    srv.request.Waypoints = Waypoints;
    srv.request.dt_flat_states.data = sampling_time;
    if (client.call(srv))
    {
        ROS_INFO("Return size: %d", int(srv.response.flatStates.PVAJS_array.size()));
        ROS_INFO("[inspector_node] Service returned succesfully with takeoff trajectory!");
    }
    else
    {
        ROS_ERROR("[inspector_node] Failed to call service ""%s"" for takeoff trajectory...",
                  client.getService().c_str());
        return false;
    }

    *flatStates = srv.response.flatStates;

    return true;
}


bool InspectorClass::CallPVAJSAction(const std::string &ns, const mg_msgs::PVAJS_array &flatStates,
                                     const double &sampling_time, const bool &wait_until_done,
                                     ros::NodeHandle *nh) {
    // Send takeoff trajectory to px4_control action that handles trajectories
    std::string action_name = "/" + ns + "/follow_PVAJS_trajectory_action";
    actionlib::SimpleActionClient<mg_msgs::follow_PVAJS_trajectoryAction> 
        followPVAJS_action_client(action_name, true);

    ros::Duration timeout(2.0);
    if(!followPVAJS_action_client.waitForServer(timeout)) {
        ROS_ERROR("[inspector_node] Failed to call action server for sending trajectory: timeout!");
        return false;
    }

    ROS_INFO("Action server started, sending goal.");
    mg_msgs::follow_PVAJS_trajectoryGoal action_msg_goal;
    action_msg_goal.samplingTime = sampling_time;
    action_msg_goal.flatStates = flatStates;
    followPVAJS_action_client.sendGoal(action_msg_goal);

    if (wait_until_done) {
        bool finished_before_timeout = followPVAJS_action_client.waitForResult();
    }

    return true;
}

}  // namespace inspector