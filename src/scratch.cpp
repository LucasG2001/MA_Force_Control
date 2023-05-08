//
// Created by lucas on 26.04.23.
//
void ForceActionController::action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal, ActionServer* server){
    ROS_INFO("got a message");
    ros::Time start_time = ros::Time::now();
    ros::Time control_time;
    size_t trajectory_length = goal->trajectory.points.size();
    trajectory_msgs::JointTrajectory trajectory = goal->trajectory;
    std::vector<control_msgs::JointTolerance> tolerances = goal->path_tolerance;
    std::vector<control_msgs::JointTolerance> goal_tolerance = goal->goal_tolerance;
    tolerances.reserve(tolerances.size() + goal_tolerance.size());
    tolerances.insert(tolerances.end(), goal_tolerance.begin(), goal_tolerance.end());
    if (tolerances.size() != trajectory_length){ ROS_INFO("tolerance size mismatch");}
    ros::Duration time_tolerance = goal->goal_time_tolerance;
    std::array<double, 7> q_ref;
    for (size_t i=0; i<trajectory_length; i++){
        std::transform(trajectory.points[i].positions.begin(), trajectory.points[i].positions.end(), q_ref.data(), [](const double &x){return x;});
        Eigen::Affine3d reference_pose(Eigen::Matrix4d::Map(model_handle_->getPose(franka::Frame::kEndEffector, q_ref, F_T_EE, EE_T_K).data()));
        translation_desired = reference_pose.translation();
        orientation_desired = Eigen::Quaterniond(reference_pose.rotation()); //pose desired or initial transform
        control_time = ros::Time::now();
        while((control_time - start_time) < trajectory.points[i].time_from_start){
            control_time = ros::Time::now();
        }
    }
    server->setSucceeded();
}