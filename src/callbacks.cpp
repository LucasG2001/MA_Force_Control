//
// Created by lucas on 13.06.23.
//
#include <force_control/force_action_controller.h>
#include <pluginlib/class_list_macros.h>
#include <chrono>
#include <fstream>
#include <functional>
#include <thread>

#define IDENTITY Eigen::MatrixXd::Identity(6,6)

namespace force_control {
    void ForceActionController::action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal, ActionServer* server){
        ROS_INFO("got a message");
        std::cout << "Thread ACTION CALLBACK ID: " << std::this_thread::get_id() << std::endl;
        ros::Rate rate = 150;

        if (goal->trajectory.joint_names[0] == "0"){
            control_mode = 0; //don't worry about Nullspace torques and set position gain to 0
            //if this is the case we switch to hybrid force/motion control
            //expect w in the joint velocity field 0-5
            //expect F in the effort fields 0-5
            ROS_INFO("Start Force Action");
            for (size_t i = 0; i < 6; i++){
                F_contact_des(i,0) = goal->trajectory.points[0].effort[i]; //set reference force of 3 Newton in negative z-direction
                w_des(i,0) = goal->trajectory.points[0].velocities[i]; //set reference velocity of 1 cm/s in y-direction
                //dq_desired << 0, 0, 0, 0, 0, 0;
                dq_desired = pinv_J * Sm * w_des;
            }

            double roll =  goal->trajectory.points[0].positions[3]; // Rotation around X-axis (in radians)
            double pitch = goal->trajectory.points[0].positions[4]; // Rotation around Y-axis (in radians)
            double yaw = goal->trajectory.points[0].positions[5];   // Rotation around Z-axis (in radians)
            /**
            Eigen::Quaterniond quaternion;
            quaternion = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
             **/
            // Convert rotation matrix to quaternion
            if (started_force_action){
                orientation_target = orientation;
                started_force_action = false;
            }

            orientation_target = orientation;
            translation_target(0,0) = goal->trajectory.points[0].positions[0]; //set x-direction
            translation_target(1,0)  = goal->trajectory.points[0].positions[1]; //set y-direction
            translation_target(2,0) = goal->trajectory.points[0].positions[2]; //set z-direction

            pose_desired = Eigen::Affine3d::Identity();
            pose_desired.rotate(orientation_target);
            pose_desired.translation() = translation_target;

            //pose_desired = current_pose;
            //translation_desired = position;


            Eigen::Matrix<double, 6, 1> force_directions;
            force_directions << 0, 0, 0, 0 ,0, 0;
            Sf = force_directions.asDiagonal();
            Sm = IDENTITY - Sf;

        }
        else{
            I_error.setZero();
            control_mode = 0;
            started_force_action = true;
            ros::Time callback_start = ros::Time::now(); // Get the trajectory points from the goal
            F_contact_des << 0, 0, 0, 0, 0, 0;
            Sm = IDENTITY;
            Sf = IDENTITY - Sm;
            std::array<double, 7> q_ref{};
            std::array<double, 7> dq_ref{};
            bool goal_reached = false;
            size_t trajectory_size = goal->trajectory.points.size();
            ROS_INFO_STREAM("trajectory_size = " << trajectory_size);
            // Loop over the trajectory points
            double tol = 0.02;
            for (size_t i = 0; i < trajectory_size; i++)
            {
                // Set the new reference orientation for amd position
                const auto& point = goal->trajectory.points[i];
                std::copy(point.positions.begin(), point.positions.end(), q_ref.begin());
                q_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_ref.data());
                std::copy(point.velocities.begin(), point.velocities.end(), dq_ref.begin());
                dq_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(dq_ref.data());
                w_des  = 0*J * dq_desired;
                pose_desired = (Eigen::Matrix4d::Map(model_handle_->getPose(franka::Frame::kEndEffector, q_ref, F_T_EE, EE_T_K).data()));
                translation_target = pose_desired.translation();
                orientation_target = Eigen::Quaterniond(pose_desired.rotation());
                if (i > 0) {
                    ROS_INFO("Adding trajectory point");
                    //I_error *= 0; //clear the integrator
                    const auto& last_point = goal->trajectory.points[i - 1];
                    ros::Duration wait_time = point.time_from_start - last_point.time_from_start;
                    ros::Time loop_start = ros::Time::now();
                    while(ros::Time::now() - loop_start < wait_time && !goal_reached) {
                        if (pose_error.head(3).norm() < tol && pose_error.tail(3).norm() < 0.15){
                            goal_reached = true;
                            //ROS_INFO_STREAM("Orientation_tolerance  = " << pose_error.tail(3).norm());
                            //ROS_INFO_STREAM("pose_error  = " << pose_error);
                        }
                        ROS_INFO_STREAM("goal reached = " << goal_reached);
                        rate.sleep();
                    }
                    goal_reached = false;
                }
                //wait for first point
                //ros::Duration callback_duration = ros::Time::now() - callback_start;
                //ROS_INFO_STREAM("will sleep for = " << callback_duration);
                //(point.time_from_start-callback_duration).sleep();
                //rate.sleep();
            }
        }
        server->setSucceeded();
    }

}
