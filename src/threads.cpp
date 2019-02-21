
#include <mission_planner/mission_class.h>

namespace mission_planner {

// Thread for constantly updating the tfTree values
void MissionClass::TfTask() {
    ROS_DEBUG("[mission_node] tf Thread started with rate %f: ", tf_update_rate_);
    tf_listener::TfClass obj_quad2world;
    ros::Rate loop_rate(tf_update_rate_);

    std::string vehicle_frame = ns_ + "/base_link";
    std::string map_frame = "map";

    while (ros::ok()) {
        // Get the transforms
        obj_quad2world.GetTransform(vehicle_frame , map_frame);

        mutexes_.tf.lock();
            globals_.tf_quad2world = obj_quad2world.transform_;
        mutexes_.tf.unlock();

        loop_rate.sleep();
    }

    ROS_DEBUG("[mission_node] Exiting tf Thread...");
}

// Consumer/producer thread
// Consume waypoints to produce minimum snap trajectories
void MissionClass::MinSnapSolverTask(const std::string &ns) {
    ROS_DEBUG("[mission_node] MinSnapSolverTask started.");

    ros::NodeHandle nh;
    ros::Rate loop_rate(10); // Runs at 10hz
    std::queue<mission_planner::minSnapWpInputs> min_snap_inputs_queue;
    mission_planner::minSnapWpInputs min_snap_input;

    // Load desired average velocity
    double max_vel = max_velocity_;

    while (ros::ok()) {
        // See if there is anything in the buffer
        mutexes_.waypoint_buffer.lock();
        	min_snap_inputs_queue = globals_.min_snap_inputs;
        mutexes_.waypoint_buffer.unlock();

        if(min_snap_inputs_queue.size() > 0) {
            min_snap_input = min_snap_inputs_queue.front();
        } else {
            loop_rate.sleep();
            continue;
        }

        // ROS_INFO("[MinSnapSolverTask] Running minimum snap problem!");

        // Solve minimum snap problem
        mission_planner::TrajectoryActionInputs traj_inputs;
        if (min_snap_input.waypoints_.size() < 2) {
            ROS_WARN("Cannot solve minimum snap: not enough waypoints!");
        } else if(min_snap_input.waypoints_.size() == 2) {
            double distance = 
                min_snap_input.waypoints_[0].GetXYZDist(min_snap_input.waypoints_[1]);
            double tf = 2.0*distance/max_vel;
            this->MinSnapPoint2Point(ns, min_snap_input.waypoints_[0],
                          min_snap_input.waypoints_[1], tf, 
                          min_snap_input.sampling_time_, &nh, &traj_inputs.flatStates);
        } else {
            this->MinSnapWaypointSet(ns, min_snap_input, &nh, &traj_inputs.flatStates);
        }
        traj_inputs.start_immediately = false;
        traj_inputs.sampling_time = min_snap_input.sampling_time_;
        traj_inputs.action_type = ActionType::Trajectory;

        mutexes_.trajectory_buffer.lock();
            globals_.traj_inputs.push_back(traj_inputs);
        mutexes_.trajectory_buffer.unlock();

        mutexes_.waypoint_buffer.lock();
            globals_.min_snap_inputs.pop();
        mutexes_.waypoint_buffer.unlock();

        loop_rate.sleep();
    }

    ROS_DEBUG("[mission_node] Exiting MinSnapSolverTask...");
}

// Consumer thread
// Consumes minimum snap trajectories
void MissionClass::TrajectoryActionCaller(const std::string &ns) {
    ROS_DEBUG("[mission_node] TrajectoryActionCaller started.");

    ros::NodeHandle nh;
    ros::Rate loop_rate(10); // Runs at 10hz
    const bool wait_until_done = false;
    bool action_server_busy;
    std::list<TrajectoryActionInputs> traj_inputs;
    TrajectoryActionInputs local_traj_inputs;

    mutexes_.quad_is_busy.lock();
        globals_.quad_is_busy = false;
    mutexes_.quad_is_busy.unlock();

    // Create action handle
    std::string action_name = "/" + ns + "/follow_PVAJS_trajectory_action";
    actionlib::SimpleActionClient<mg_msgs::follow_PVAJS_trajectoryAction> 
    		followPVAJS_action_client(action_name, true);

    while (ros::ok()) {
        // Check if there is anything in the buffer
        mutexes_.trajectory_buffer.lock();
        	traj_inputs = globals_.traj_inputs;
        mutexes_.trajectory_buffer.unlock();

        // Get current state of the action server
        actionlib::SimpleClientGoalState status = followPVAJS_action_client.getState();
        if (status.state_ == status.ACTIVE) {
            action_server_busy = true;
        } else {
            action_server_busy = false;
        }

        // Set global variable of whether server is busy or not
        mutexes_.quad_is_busy.lock();
            globals_.quad_is_busy = action_server_busy;
        mutexes_.quad_is_busy.unlock();

        if(traj_inputs.size() > 0) {
        	// Retrieve first item in the buffer
            local_traj_inputs = traj_inputs.front();
        } else {
            loop_rate.sleep();
            continue;
        }

        // Send data to action server (start immediately or wait until server is no longer active)
        if (local_traj_inputs.start_immediately) {
            // Start immediately is usually triggered when unsafe behaviour is detected
            followPVAJS_action_client.cancelAllGoals();
            this->CallActionType(ns, local_traj_inputs, wait_until_done, 
                                 &nh, &followPVAJS_action_client);

        } else {
            if(!action_server_busy) {
                mutexes_.quad_is_busy.lock();
                    globals_.quad_is_busy = true;
                mutexes_.quad_is_busy.unlock();
                // Start only when server is no longer busy (also removes new trajectory from buffer)
                this->CallActionType(ns, local_traj_inputs, wait_until_done, 
                                 &nh, &followPVAJS_action_client);
            }
        }

        loop_rate.sleep();
    }

    ROS_DEBUG("[mission_node] Exiting TrajectoryActionCaller...");
}

}  // namespace mission_planner