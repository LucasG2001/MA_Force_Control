//
// Created by lucas on 13.06.23.
//
#include <force_control/force_action_controller.h>
#include <force_control/cartesian_impedance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <chrono>
#include <fstream>
#include <functional>
#include <thread>


#define IDENTITY Eigen::MatrixXd::Identity(6,6)
#define ZERO Eigen::MatrixXd::Zero(6,6)

namespace force_control {
    void ForceActionController::action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal,
                                                ActionServer *server) {
        ROS_INFO("got a message");
        std::cout << "Thread ACTION CALLBACK ID: " << std::this_thread::get_id() << std::endl;
        ros::Rate rate = 150;

        if (goal->trajectory.joint_names[0] == "0") {
            control_mode = 0; //don't worry about Nullspace torques and set position gain to 0
            //if this is the case we switch to hybrid force/motion control
            //expect w in the joint velocity field 0-5
            //expect F in the effort fields 0-5
            ROS_INFO("Start Force Action");
            for (size_t i = 0; i < 6; i++) {
                F_contact_des(i,
                              0) = goal->trajectory.points[0].effort[i]; //set reference force of 3 Newton in negative z-direction
                w_des(i,
                      0) = goal->trajectory.points[0].velocities[i]; //set reference velocity of 1 cm/s in y-direction
                //dq_desired << 0, 0, 0, 0, 0, 0;
                dq_desired = pinv_J * Sm * w_des;
            }

            double roll = goal->trajectory.points[0].positions[3]; // Rotation around X-axis (in radians)
            double pitch = goal->trajectory.points[0].positions[4]; // Rotation around Y-axis (in radians)
            double yaw = goal->trajectory.points[0].positions[5];   // Rotation around Z-axis (in radians)
            /**
            Eigen::Quaterniond quaternion;
            quaternion = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
             **/
            // Convert rotation matrix to quaternion
            if (started_force_action) {
                orientation_target = orientation;
                started_force_action = false;
            }

            orientation_target = orientation;
            translation_target(0, 0) = goal->trajectory.points[0].positions[0]; //set x-direction
            translation_target(1, 0) = goal->trajectory.points[0].positions[1]; //set y-direction
            translation_target(2, 0) = goal->trajectory.points[0].positions[2]; //set z-direction

            pose_desired = Eigen::Affine3d::Identity();
            pose_desired.rotate(orientation_target);
            pose_desired.translation() = translation_target;

            //pose_desired = current_pose;
            //translation_desired = position;


            Eigen::Matrix<double, 6, 1> force_directions;
            force_directions << 0, 0, 0, 0, 0, 0;
            Sf = force_directions.asDiagonal();
            Sm = IDENTITY - Sf;

        } else {
            I_error.setZero();
            q_I_error.setZero();
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
            double tol = 0.025;
            for (size_t i = 0; i < trajectory_size; i++) {
                // Set the new reference orientation for amd position
                const auto &point = goal->trajectory.points[i];
                std::copy(point.positions.begin(), point.positions.end(), q_ref.begin());
                q_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_ref.data());
                std::copy(point.velocities.begin(), point.velocities.end(), dq_ref.begin());
                dq_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(dq_ref.data());
                w_des = 0 * J * dq_desired;
                pose_desired = (Eigen::Matrix4d::Map(
                        model_handle_->getPose(franka::Frame::kEndEffector, q_ref, F_T_EE, EE_T_K).data()));
                translation_target = pose_desired.translation();
                orientation_target = Eigen::Quaterniond(pose_desired.rotation());
                if (i > 0) {
                    ROS_INFO("Adding trajectory point");
                    //I_error *= 0; //clear the integrator
                    const auto &last_point = goal->trajectory.points[i - 1];
                    ros::Duration wait_time = point.time_from_start - last_point.time_from_start;
                    ros::Time loop_start = ros::Time::now();
                    while (ros::Time::now() - loop_start < wait_time && !goal_reached) {
                        if (pose_error.head(3).norm() < tol && pose_error.tail(3).norm() < 0.5) {
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


    void CartesianImpedanceController::equilibriumPoseCallback(
            const geometry_msgs::PoseStampedConstPtr &msg) {
            I_error = Eigen::MatrixXd::Zero(6,1); //clear integrator
            Sm = IDENTITY;
            Sf = ZERO;
            config_control = false;
            std::lock_guard<std::mutex> position_d_target_mutex_lock(
                    position_and_orientation_d_target_mutex_);
            position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
			// Safety regulation (sanity check to keep desired position inside of workspace, length of panda robot is approx 1.12m)
			if (position_d_target_.norm() > 0.85){
				position_d_target_ = (0.85/position_d_target_.norm()) * position_d_target_;
				ROS_INFO("Desired Position is out of Workspace bounds");
			}
            Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
            orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w;
            if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
                orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
            }
            ROS_INFO_STREAM("new reference pose is" << position_d_target_.transpose() << "\n" << orientation_d_target_.coeffs());
    } //callback

    void CartesianImpedanceController::control_mode_callback(const std_msgs::Int16ConstPtr &msg) {
        I_error = Eigen::MatrixXd::Zero(6,1); //clear integrator
        ROS_INFO("switching control mode");
        control_mode = msg->data;
        if(control_mode == 1){
            K.setZero(); D.setZero(); repulsion_K.setZero(); repulsion_D.setZero(); nullspace_stiffness_target_ = 0; //this wil also effectively set F_repulsion to 0
            cartesian_stiffness_target_.setZero(); cartesian_damping_target_.setZero();
            F_contact_target *= 0;
            Sf = ZERO;
            Sm = IDENTITY; //delete all previous forcing commands
        }
        else if (control_mode == 0){
            nullspace_stiffness_target_ = 0.0001;
            cartesian_stiffness_target_.topLeftCorner(3, 3) = 250 * Eigen::Matrix3d::Identity();
            cartesian_stiffness_target_.bottomRightCorner(3, 3) << 60, 0, 0, 0, 60, 0, 0, 0, 10;
            cartesian_damping_target_.topLeftCorner(3, 3) = 55 * Eigen::Matrix3d::Identity();
            cartesian_damping_target_.bottomRightCorner(3, 3) << 18, 0, 0, 0, 18, 0, 0, 0, 6;
        }

    }

	//currently unused
    void CartesianImpedanceController::complianceParamCallback(
            franka_example_controllers::compliance_paramConfig &config,
            uint32_t /*level*/) {
        //ToDo: add update for T
        cartesian_stiffness_target_.setIdentity();
        cartesian_stiffness_target_.topLeftCorner(3, 3)
                << config.translational_stiffness * Eigen::Matrix3d::Identity();
        cartesian_stiffness_target_.bottomRightCorner(3, 3)
                << config.rotational_stiffness * Eigen::Matrix3d::Identity();
        cartesian_damping_target_.setIdentity();
        // Damping ratio = 1
        cartesian_damping_target_.topLeftCorner(3, 3)
                << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
        cartesian_damping_target_.bottomRightCorner(3, 3)
                << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
        nullspace_stiffness_target_ = config.nullspace_stiffness;
    }

    void CartesianImpedanceController::JointConfigCallback(const sensor_msgs::JointState &goal) {
        ROS_INFO("received joint position goal");
        for (int i = 0; i < 7; i++) {
            q_d_nullspace_(i, 0) = goal.position[i];
        }

    }

    void CartesianImpedanceController::HandPoseCallback(const geometry_msgs::Point &right_hand_pos) {
		do_logging = true;
        // ROS_INFO("received hand position");
        R = 0.25;

        C.x() = 0.5 * right_hand_pos.x + 0.5 * C.x(); //smoothing
        C.y() = 0.5 * right_hand_pos.y + 0.5 * C.y();
        C.z() = 0.5 * right_hand_pos.z + 0.5 * C.z();


    }

    void CartesianImpedanceController::action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal,
                                                       ActionServer *server) {
        I_error = Eigen::MatrixXd::Zero(6,1);; //clear integrator
        ROS_INFO("got a message");
        do_logging = false;
        count = 0; //reset count time
        config_control = true; // do explicitly control q-configuration in nullspace
        ros::Rate rate = 150;
        ros::Time callback_start = ros::Time::now(); // Get the trajectory points from the goal
        F_contact_target << 0, 0, 0, 0, 0, 0;
        filter_params_ = 0.999; //remove filtering of position reference during moveit trajectory
        Sm = IDENTITY;
        Sf = IDENTITY - Sm;
        std::array<double, 7> q_ref{}, dq_ref{};
        bool goal_reached = false;

        size_t trajectory_size = goal->trajectory.points.size();
        // Loop over the trajectory points
        double tol = 0.025;
        for (size_t i = 0; i < trajectory_size; i++) {
            // Set the new reference orientation for amd position
            const auto &point = goal->trajectory.points[i];
            std::copy(point.positions.begin(), point.positions.end(), q_ref.begin());
            q_d_nullspace_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_ref.data());
            std::copy(point.velocities.begin(), point.velocities.end(), dq_ref.begin());
            //ToDo: Implement Velocity component for cartesian impedance (is it even useful?)
            //dq_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(dq_ref.data());
            // w_des  =  J * dq_desired;
            pose_desired = (Eigen::Matrix4d::Map(
                    model_handle_->getPose(franka::Frame::kEndEffector, q_ref, F_T_EE, EE_T_K).data()));
            position_d_target_ = pose_desired.translation();
            orientation_d_target_ = Eigen::Quaterniond(pose_desired.rotation());
            if (i > 0) {
                const auto &last_point = goal->trajectory.points[i - 1];
                ros::Duration wait_time = point.time_from_start - last_point.time_from_start;
                ros::Time loop_start = ros::Time::now();
                while (ros::Time::now() - loop_start < wait_time && !goal_reached) {
                    if (error.head(3).norm() < tol && error.tail(3).norm() < 0.5) {
                        goal_reached = true;
                    }
                    ROS_INFO_STREAM("goal reached = " << goal_reached);
                    rate.sleep();
                }
                goal_reached = false;
            }
        }

        server->setSucceeded();
        filter_params_ = 0.005; //set filter back
    }

    void CartesianImpedanceController::force_callback(const geometry_msgs::PoseStampedConstPtr &goal_pose) {
        /**  **/
        I_error *= 0.0;
        ROS_INFO("got a message: apply force");
        do_logging = true;
        //reset file
        /**
        std::ofstream F;
        F.open("/home/lucas/Desktop/MA/Force_Data/F_corrections.txt");
        F << "time Fref F_cmd Fx Fy Fz Mx My Mz F_imp_x F_imp_y F_imp_z F_imp_Mx F_imp_My F_imp_Mz\n";
        F.close();
         **/
        config_control = false; // do not explicitly control q-configuration in nullspace
        Eigen::Matrix<double, 6, 1> force_directions; force_directions << 0, 0, 1, 0, 0, 0; //apply force in negative z-direction
        F_contact_target = force_directions * -10.0;
        Sf = force_directions.asDiagonal();
        Sm = IDENTITY - Sf;
        position_d_target_ << goal_pose->pose.position.x, goal_pose->pose.position.y, goal_pose->pose.position.z;
        ROS_INFO_STREAM("Forcing Goal Pose is " << position_d_target_.transpose());
        Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
        orientation_d_target_.coeffs() << goal_pose->pose.orientation.x, goal_pose->pose.orientation.y,
                goal_pose->pose.orientation.z, goal_pose->pose.orientation.w;
        if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
            orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
        }

    }

    void CartesianImpedanceController::potential_field_callback(const geometry_msgs::Vector3 &resulting_force) {
        //ROS_INFO("got a message: updating potential field");
        F_potential.x() = 0.2* resulting_force.x + 0.8 * F_potential.x(); //smooth
        F_potential.y() = 0.2* resulting_force.y + 0.8 * F_potential.y();
        F_potential.z() = 0.2* resulting_force.z + 0.8 * F_potential.z();

    }

	void CartesianImpedanceController::impedance_param_reconfigure_callback(const custom_msgs::ImpedanceParameterMsgConstPtr &msg){
		//cartesian impedance general
		ROS_INFO("Updating Impedance Parameters");
		cartesian_stiffness_target_ = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg->stiffness.data());
		cartesian_damping_target_ = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg->damping.data());
		//cartesian_inertia_target_ = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg->stiffness.data());
		//at the moment we cannot use variable inertia
		//safety bubble
		//ToDo: Implement logic to have desired equilibrium radius
		Eigen::Matrix<double, 6, 6> bubble_stiffness = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg->safety_bubble_stiffness.data());
		Eigen::Matrix<double, 6, 6> bubble_damping = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg->safety_bubble_damping.data());
		repulsion_K_target_ = bubble_stiffness.topLeftCorner(3,3);
		repulsion_D_target_ = bubble_damping.topLeftCorner(3,3);
		std::cout << "new bubble stiffness is " << repulsion_K_target_;
		std::cout << "new bubble damping is " << repulsion_D_target_;
	}
} //namespace force_control
