

// Standard includes
#include <ros/ros.h>
#include <mission_planner/inspection_class.h>

int main(int argc, char **argv) {

	ROS_INFO("[mission_node]: Starting...");

	ros::init(argc, argv, "mission_node");
	ros::NodeHandle node("~");

	// new_class_example example_obj;
	// example_obj.initialize_example();
	
	inspector::InspectorClass mission_obj;
	mission_obj.ShelfMission(&node);

	ros::spin();

	return 0;
}