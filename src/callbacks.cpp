

#include <mission_planner/mission_class.h>

namespace mission_planner {

void MissionClass::ActionGoalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) {
	mutexes_.action_server_status.lock();
		globals_.action_server_status = *msg;
	mutexes_.action_server_status.unlock();
}

}  // namespace mission_planner