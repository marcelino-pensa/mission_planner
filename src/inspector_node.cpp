

// Standard includes
#include <ros/ros.h>
#include <inspector/inspector_class.h>

int main(int argc, char **argv) {

	ROS_INFO("[inspector_node]: Starting...");

	ros::init(argc, argv, "inspector_node");
	ros::NodeHandle node("~");
	
	inspector::InspectorClass inspector_obj;
	inspector_obj.Initialize(&node);

	ros::spin();

	return 0;
}