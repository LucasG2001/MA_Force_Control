//
// Created by lucas on 17.04.23.
//

#include <force_control/force_action_controller.h>
#include <pluginlib/class_list_macros.h>
#include <chrono>
#include <fstream>
#include <functional>
#include <thread>


#define IDENTITY Eigen::MatrixXd::Identity(6,6)

namespace force_control {

    namespace {
        template <class T, size_t N>
        std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
            //ostream << "[";
            std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
            std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
            //ostream << "]";
            return ostream;
        }
    }  // anonymous namespace

    inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
        double lambda_ = damped ? 0.2 : 0.0;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
        Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
        S_.setZero();

        for (int i = 0; i < sing_vals_.size(); i++)
            S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

        M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
    }

    void ForceActionController::action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal, ActionServer* server){
        ROS_INFO("got a message");
        std::cout << "Thread ACTION CALLBACK ID: " << std::this_thread::get_id() << std::endl;
        additional_task = 1.0;
        ros::Rate rate = 500;
        if (goal->trajectory.joint_names[0] == "0"){
            //if this is the case we switch to hybrid force/motion control
            //expect w in the joint velocity field 0-5
            //expect F in the effort fields 0-5
            franka::RobotState state = franka_state_handle_->getRobotState();
            ROS_INFO("Start Force Action");
            q_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(state.q.data());
            F_contact_des(2,1) = -3.0 ; //set reference force of 3 Newton in negative z-direction
            w_des << 0, 0.1, 0, 0, 0, 0; //set reference velocity of 1 cm/s in y-direction
            force_control::pseudoInverse(J_T, pinv_J_T);
            dq_desired = pinv_J_T * w_des;
            additional_task = 0.0; //don't worry about Nullspace torques and set position gain to 0
            pose_desired = Eigen::Matrix4d::Map(franka_state_handle_->getRobotState().O_T_EE.data());
            translation_desired = pose_desired.translation() + 1 * dt * w_des.head(0);
            orientation_desired = Eigen::Quaterniond(pose_desired.rotation());
            Eigen::Matrix<double, 6, 1> force_directions;
            force_directions << 0, 0, 1, 0 ,0, 0;
            Sf = force_directions.asDiagonal();
            Sm = IDENTITY - Sf;

        }
        else{
            ros::Time callback_start = ros::Time::now();
            // Get the trajectory points from the goal
            F_contact_des << 0, 0, 0, 0, 0, 0;
            Sm = IDENTITY;
            Sf = IDENTITY - Sm;
            std::array<double, 7> q_ref{};
            std::array<double, 7> dq_ref{};
            bool goal_reached = false;
            size_t trajectory_size = goal->trajectory.points.size();
            // Loop over the trajectory pointsxkil
            for (size_t i = 0; i < trajectory_size; i++)
            {
                // Set the new reference orientation for amd position
                const auto& point = goal->trajectory.points[i];
                std::copy(point.positions.begin(), point.positions.end(), q_ref.begin());
                q_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_ref.data());
                double q_tol = 0.01;
                std::copy(point.velocities.begin(), point.velocities.end(), dq_ref.begin());
                dq_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(dq_ref.data());
                w_des  = J * dq_desired;
                pose_desired = (Eigen::Matrix4d::Map(model_handle_->getPose(franka::Frame::kEndEffector, q_ref, F_T_EE, EE_T_K).data()));
                translation_desired = pose_desired.translation();
                orientation_desired = Eigen::Quaterniond(pose_desired.rotation());
                if (i > 0) {
                    const auto& last_point = goal->trajectory.points[i - 1];
                    ros::Duration wait_time = point.time_from_start - last_point.time_from_start;
                    ros::Time loop_start = ros::Time::now();
                    while(ros::Time::now() - loop_start < wait_time && !goal_reached) {
                        goal_reached = true;
                        for(size_t k = 0; k < 7; k++){
                            bool temp = (std::abs(q_desired(k) - q(k)) < q_tol); //true if q_des_k - q_k < tol, else false
                            goal_reached = goal_reached && temp; //if all joints reached the goal this is true (true&&tre&&true...)
                        }
                        rate.sleep();
                    }
                }
                //wait for first point
                ros::Duration callback_duration = ros::Time::now() - callback_start;
                (point.time_from_start-callback_duration).sleep();
            }
        }
        server->setSucceeded();
    }


    bool ForceActionController::init(hardware_interface::RobotHW *robot_hw,
                                     ros::NodeHandle &node_handle) {

        std::vector<std::string> joint_names;
        franka_state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (franka_state_interface_ == nullptr) {
            ROS_ERROR("ModelExampleController: Could not get Franka state interface from hardware");
            return false;
        }
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("ModelExampleController: Could not read parameter arm_id");
            return false;
        }
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
            ROS_ERROR(
                    "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
                    "controller init!");
            return false;
        }
        model_interface_ = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface_ == nullptr) {
            ROS_ERROR_STREAM("ModelExampleController: Error getting model interface from hardware");
            return false;
        }

        try {
            franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                    franka_state_interface_->getHandle(arm_id + "_robot"));
        } catch (const hardware_interface::HardwareInterfaceException &ex) {
            ROS_ERROR_STREAM(
                    "ModelExampleController: Exception getting franka state handle: " << ex.what());
            return false;
        }

        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                    model_interface_->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException &ex) {
            ROS_ERROR_STREAM(
                    "ModelExampleController: Exception getting model handle from interface: " << ex.what());
            return false;
        }
        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }
        return true;
    }

    void ForceActionController::starting(const ros::Time& /*time*/) {

        franka::RobotState initial_state = franka_state_handle_->getRobotState();
        current_pose = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(initial_state.O_T_EE.data()); //initital pose matrix
        k_gains_ << 600, 600, 600, 600, 200, 100, 350;
        d_gains_ << 50, 50, 50, 20, 20, 20, 10;
        F_T_EE = initial_state.F_T_EE;
        EE_T_K = initial_state.EE_T_K;
        std::ofstream F;
        F.open("/home/lucas/Desktop/F.txt");
        F << "time Fx Fy Fz Mx My Mz\n";
        F.close();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        pose_desired =(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        // set equilibrium point to current state
        translation_desired = pose_desired.translation();
        orientation_desired = Eigen::Quaterniond(pose_desired.rotation());
        // set nullspace equilibrium configuration to initial q
        q_desired = q_initial;
        dq_desired << 0, 0, 0, 0, 0, 0, 0;
        F_bias = Eigen::Map<Eigen::Matrix<double, 6, 1>>(initial_state.O_F_ext_hat_K.data()); //F_joints - F_joints_desired - gravity
    }

    void ForceActionController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
        if (rate_trigger_()) {

            auto start = std::chrono::high_resolution_clock ::now();
            if (count == 1){
                std::cout << "Thread MAIN ID: " << std::this_thread::get_id() << std::endl;
            }

            update_state();
            Eigen::Matrix<double, 6, 1> error = get_pose_error();
            get_dynamic_model_parameters();
            compute_task_space_matrices(),
            //get current EE-velocity
            w = J * q_dot;
            I_error += error;
            //compute PD-control laq acceleration in operational space
            Eigen::Matrix<double, 6, 1> pose_gains;
            pose_gains << 90, 90, 90, 180, 180, 180;
            Eigen::Matrix<double, 6, 6> kp = pose_gains.asDiagonal();
            double kd = 2*std::sqrt(kp(1,1)) * 1.1; //select kd to slightly under-damp
            double ki = 0.05;
            //calculate desired acceleration with PD-control law
            w_dot_des = kp * error + ki * I_error + kd * (w_des - w);

            Eigen::Matrix<double, 6, 1> F_ext = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state_.O_F_ext_hat_K.data());
            delta_F = F_contact_des + F_ext - F_bias; //F_ext shows in the negative direction of F_contact desired
            error_F += dt * delta_F;
            F_last = F_ext;
            Eigen::Matrix<double, 6, 1> F_cmd = F_contact_des + 100 * delta_F + 8 * error_F;

            Eigen::Matrix<double, 7, 7> Kp = k_gains_.asDiagonal();
            Eigen::Matrix<double, 7, 7> Kd = d_gains_.asDiagonal();
            Eigen::Matrix<double, 7, 1> tau_nullspace = add_nullspace_torque(Kp, Kd);

            tau_des = J_T * (Lambda * Sm * w_dot_des + Sf * F_cmd + mu); // no need for g because franka compensates it automatically
            //update torque signal with second task
            tau_des = tau_des + tau_nullspace * additional_task;
            check_movement_and_log(false);

            Eigen::Matrix<double, 7, 1> tau_measured = Eigen::Map<Eigen::Matrix<double, 7 , 1>>(robot_state_.tau_J.data());
            for (size_t i = 0; i < 7; ++i) {
                tau_des = saturateTorqueRate(tau_des, tau_measured);
                joint_handles_[i].setCommand(std::min(tau_des(i), 87.0));
            }

            auto stop = std::chrono::high_resolution_clock::now();
            auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            //std::cout << "Time taken by update: "
            // << time.count() << " microseconds" << std::endl;
        }

    }


} //namespace Force control

PLUGINLIB_EXPORT_CLASS(force_control::ForceActionController, controller_interface::ControllerBase)