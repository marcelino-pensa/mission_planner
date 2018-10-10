

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
    ROS_DEBUG("tf Thread started with rate %f: ", tf_update_rate_);
    tf_listener::TfClass obj_quad2world;
    // TfClass obj_haz2body;
    ros::Rate loop_rate(tf_update_rate_);

    while (ros::ok()) {
        // Get the transforms
        obj_quad2world.GetTransform(ns_ + "/base_link", "map");
        // obj_haz2body.getTransform("/haz_cam", "/body");

        mutexes_.tf.lock();
            globals_.tf_quad2world = obj_quad2world.transform_;
        mutexes_.tf.unlock();

        // obj_haz2body.printTransform();
        loop_rate.sleep();
    }

    ROS_DEBUG("Exiting tf Thread...");
}

void InspectorClass::Initialize(ros::NodeHandle *nh) {
	// Get namespace of current node
	nh->getParam("namespace", ns_);
	ROS_INFO("namespace: %s", ns_.c_str());

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
		ROS_INFO("Localization waypoints were loaded successfully. Number of waypoints: %d",
		         static_cast<int>(inspection_waypoint_list_.size()) );
	}

	// Load inspection file
	if(!LoadWaypoints(inspection_file, &localization_waypoint_list_)) {
		return;
	} else {
		ROS_INFO("Inspection waypoints were loaded successfully. Number of waypoints: %d",
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
	ROS_INFO("Quad initial pose:");
	tf_initial_pose.PrintTransform();


	// Plan a minimum snap trajectory for taking-off
	nav_msgs::Path Waypoints;
	geometry_msgs::PoseStamped Pos0, Pos_final;
	double tf = takeoff_height/avg_velocity;

	ros::ServiceClient client = nh->serviceClient<mav_trajectory_generation_ros::minSnapStamped>("/minSnap");
	mav_trajectory_generation_ros::minSnapStamped srv;
	Pos0.pose.position = tf_initial_pose.transform_.getOrigin();
	Pos0.header.stamp = ros::Time(0.0);
	Pos_final.pose.position = tf_initial_pose.transform_.getOrigin();
	Pos_final.pose.position.z = Pos_final.pose.position.z + takeoff_height;
	Pos_final.header.stamp = ros::Time(tf);

	// Set vehicle to listen to position commands
	this->SetQuadPosMode(ns_, nh);

}

bool InspectorClass::LoadWaypoints(std::string &filename,
	                               std::vector<xyz_heading> *waypoint_list) {
	ROS_INFO("Opening waypoints file: \n%s\n", filename.c_str());
	std::ifstream myfile(filename.c_str());
	float x, y, z, yaw;

	// Check whether file could be opened (path might be wrong)
	if (myfile.is_open()) {
		while( myfile >> x >> y >> z >> yaw) {
			waypoint_list->push_back(xyz_heading(x, y, z, inspector::deg2rad(yaw)));
			// std::cout << x << " " << y << " " << z << " " << yaw << std::endl;
		}
		myfile.close();

		// Check if any waypoint was loaded
		if(waypoint_list->size() > 0) {
			return 1;
		} else {
			ROS_ERROR("No waypoints within the file: %s", filename.c_str());
			return 0;
		}
	} else {
		ROS_ERROR("Unable to open file in %s", filename.c_str());
		return 0;
	}
}

void InspectorClass::SetQuadPosMode(std::string &ns, ros::NodeHandle *nh) {
	ROS_INFO("Requesting Vehicle to Start Position Mode! This requires px4_control_node to be executing!");

	std::string service_name = "/" + ns + "/px4_control_node/setQuadPVAMode";
	ros::ServiceClient client = nh->serviceClient<std_srvs::Trigger>(service_name);
	
	if(!client.waitForExistence(ros::Duration(1.0))) {
		ROS_ERROR("Service ""%s"" unavailable for call.", client.getService().c_str());
	}

	std_srvs::Trigger trigger_msg;
	if (client.call(trigger_msg)) {
		if(trigger_msg.response.success == false) {
			ROS_ERROR("Failed to set vehicle to listen to position commands.");
		}
	} else {
		ROS_ERROR("Failed to call service.");
	}
}


}  // namespace inspector