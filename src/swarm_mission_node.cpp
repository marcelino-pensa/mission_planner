

// Standard includes
#include <ros/ros.h>
#include <mission_planner/vicon_swarm_inspection.h>

int main(int argc, char **argv) {

	ROS_INFO("[mission_node]: Starting...");

	ros::init(argc, argv, "mission_node");
	ros::NodeHandle node("~");
	
	// inspector::InspectorClass mission_obj;
	// mission_obj.Mission(&node);

	std::string ns = "Gollum";
	double tf_update_rate = 10;
	double max_velocity = 0.20;
	double max_acc = 0.20;
	uint rover_index = 2;
	rover_planner::RoverMissionClass rover(ns, tf_update_rate, max_velocity, rover_index);
	
	std::vector<Eigen::Vector2d> wps;
	wps.push_back(Eigen::Vector2d(0.0, 0.0));
	wps.push_back(Eigen::Vector2d(0.5, 0.2));
	wps.push_back(Eigen::Vector2d(0.0, 0.5));
	Eigen::Vector2d init_vel = Eigen::Vector2d(0.0, 0.0);
	Eigen::Vector2d final_vel = Eigen::Vector2d(0.0, 0.0);
	Eigen::Vector2d final_wp;

	ros::Duration(1.0).sleep();
	rover.AddWaypoints2Buffer(wps, init_vel, final_vel, max_velocity, max_acc, "traj", &final_wp);


	ros::spin();

	return 0;
}