

// Standard includes
#include <ros/ros.h>
#include <mission_planner/inspection_class.h>

// #include <mission_planner/xyz_heading.h> 
// class minTimeWpInputs {
//  public:
// 	std::vector<mission_planner::xyz_heading> waypoints_;
// 	double max_vel_;
// 	double max_acc_;
// 	double max_jerk_;
// 	double sampling_time_;
// 	std::string name_;

// 	minTimeWpInputs() {};
// 	minTimeWpInputs(const std::vector<mission_planner::xyz_heading> &waypoints, const double &max_vel,
// 					const double &max_acc, const double &max_jerk, const double &sampling_time) {
// 		waypoints_ = waypoints;
// 		max_vel_ = max_vel;
// 		max_acc_ = max_acc;
// 		max_jerk_ = max_jerk;
// 		sampling_time_ = sampling_time;
// 		name_ = "";
// 	}
// 	minTimeWpInputs(const std::vector<mission_planner::xyz_heading> &waypoints, const double &max_vel,
// 		            const double &max_acc, const double &max_jerk,
// 		            const double &sampling_time, const std::string &name) {
// 		waypoints_ = waypoints;
// 		max_vel_ = max_vel;
// 		max_acc_ = max_acc;
// 		max_jerk_ = max_jerk;
// 		sampling_time_ = sampling_time;
// 		name_ = name;
// 	}

// 	// Create an action of Halt (name = "Halt") or Disarm (name = "Disarm")
// 	minTimeWpInputs(const std::string name) {
// 		name_ = name;
// 	}
// };

// class new_class_example {
// public:
//  std::queue<minTimeWpInputs> queue_example;
//  std::thread h_thread_;
// 	new_class_example() {
// 	}
// 	~new_class_example() {}

// 	void initialize_example() {
// 		std::cout << "queue size constructor:" << queue_example.size() << std::endl;
// 		h_thread_ = std::thread(&new_class_example::thread_function, this);
// 	}

// 	void thread_function() {
// 		std::cout << "queue size:" << queue_example.size() << std::endl;
// 	}
	
// };

int main(int argc, char **argv) {

	ROS_INFO("[mission_node]: Starting...");

	ros::init(argc, argv, "mission_node");
	ros::NodeHandle node("~");

	// new_class_example example_obj;
	// example_obj.initialize_example();
	
	inspector::InspectorClass mission_obj;
	mission_obj.Mission(&node);

	ros::spin();

	return 0;
}