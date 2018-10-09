

// Standard includes
#include <inspector/inspector_class.h>

// ---------------------------------------------------
namespace inspector {

InspectorClass::InspectorClass() {

}

InspectorClass::~InspectorClass() {
    // // Join all threads
    // h_haz_tf_thread_.join();
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

void InspectorClass::Initialize(ros::NodeHandle *nh) {
	// Get namespace of current node
	std::string ns = ros::this_node::getNamespace();

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

	// Set vehicle to listen to position commands
	this->SetQuadPosMode(ns, nh);

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
	ROS_INFO("Requesting Vehicle to Start Position Mode!");

	std::string service_name = ns + "/px4_control_node/setQuadPVAMode";
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