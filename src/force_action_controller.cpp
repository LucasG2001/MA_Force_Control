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
        additional_task = 1.0;
        if (goal->trajectory.joint_names[0] == "0"){
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
            // Loop over the trajectory pointsxkil
            for (size_t i = 0; i < goal->trajectory.points.size(); i++)
            {
                // Set the new reference orientation for amd position
                const auto& point = goal->trajectory.points[i];
                std::copy(point.positions.begin(), point.positions.end(), q_ref.begin());
                q_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_ref.data());
                std::copy(point.velocities.begin(), point.velocities.end(), dq_ref.begin());
                dq_desired = Eigen::Map<Eigen::Matrix<double, 7, 1>>(dq_ref.data());
                w_des  = J * dq_desired;
                pose_desired = (Eigen::Matrix4d::Map(model_handle_->getPose(franka::Frame::kEndEffector, q_ref, F_T_EE, EE_T_K).data()));
                translation_desired = pose_desired.translation();
                orientation_desired = Eigen::Quaterniond(pose_desired.rotation());
                if (i > 0) {
                    const auto& last_point = goal->trajectory.points[i - 1];
                    ros::Duration wait_time = point.time_from_start - last_point.time_from_start;
                    wait_time.sleep();
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
        J_last = Eigen::Map<Eigen::Matrix<double, 6, 7>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());
        dJ = Eigen::MatrixXd::Zero(6,7);
        franka::RobotState initial_state = franka_state_handle_->getRobotState();
        q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
        q_desired = q;
        pose = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(initial_state.O_T_EE.data()); //initital pose matrix
        k_gains_ << 600, 600, 600, 600, 200, 100, 350;
        d_gains_ << 50, 50, 50, 20, 20, 20, 10;
        F_T_EE = initial_state.F_T_EE;
        EE_T_K = initial_state.EE_T_K;
        std::ofstream F;
        F.open("/home/lucas/Desktop/F.txt");
        F << "time Fx Fy Fz Mx My Mz\n";
        F.close();
        return true;
    }

    void ForceActionController::starting(const ros::Time& /*time*/) {
        franka::RobotState initial_state = franka_state_handle_->getRobotState();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        pose_desired =(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        // set equilibrium point to current state
        translation_desired = pose_desired.translation();
        orientation_desired = Eigen::Quaterniond(pose_desired.rotation());
        // set nullspace equilibrium configuration to initial q
        q_desired = q_initial;
        Eigen::Matrix<double, 7, 1> g_init = Eigen::Map<Eigen::Matrix<double, 7, 1>>((model_handle_->getGravity()).data());
        //tau_bias = -Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state.tau_J.data()) - g_init;
        F_bias = Eigen::Map<Eigen::Matrix<double, 6, 1>>(initial_state.O_F_ext_hat_K.data());
    }

    void ForceActionController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
        if (rate_trigger_()) {

            auto start = std::chrono::high_resolution_clock ::now();
            //get robot state
            franka::RobotState robot_state = franka_state_handle_->getRobotState();
            q_dot = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data ());
            q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());

            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());
            Eigen::Quaterniond orientation(transform.rotation());
            // position error
            Eigen::Matrix<double, 6, 1> error;
            error.head(3) << translation_desired - position;
            // orientation error
            if (orientation_desired.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();
            }
            // "difference" quaternion
            Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_desired);
            error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            // Transform to base frame
            error.tail(3) << transform.rotation() * error.tail(3);

            //get dynamic model parameters
            std::array<double, 49> mass = model_handle_->getMass();
            std::array<double, 7> coriolis = model_handle_->getCoriolis();
            std::array<double, 7> gravity = model_handle_->getGravity();
            std::array<double, 42> ee_zero_Jac =
                    model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

            // set dynamic model parameters into Eigen::Matrices
            M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass.data());
            b = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis.data());
            g = Eigen::Map<Eigen::Matrix<double, 7, 1>>(gravity.data());
            M_inv = M.inverse();


            //get current timestamp and compute derivative of Jacobian numerically
            J = Eigen::Map<Eigen::Matrix<double, 6, 7>>(ee_zero_Jac.data());
            J_T = J.transpose();
            dJ = (J - J_last)/dt;
            J_last = J;
           //get pseudo inverse jacobian
            force_control::pseudoInverse(J_T, pinv_J_T);
            Eigen::MatrixXd pinv_J;
            force_control::pseudoInverse(J, pinv_J);

            Lambda = (J * M_inv * J_T).inverse();
            mu = Lambda * (J * M_inv * b - dJ * q_dot);
            p = Lambda * J * M_inv * g;

            /** ToDo: Implement selection matrices **/

            //get current EE-velocity
            w = J * q_dot;
            I_error += error;
            //compute PD-control laq acceleration in operational space
            Eigen::Matrix<double, 6, 1> pose_gains;
            pose_gains << 90, 90, 90, 90, 90, 90;
            Eigen::Matrix<double, 6, 6> kp = pose_gains.asDiagonal();
            double kd = 2*std::sqrt(kp(1,1)) * 1.1; //select kd to slightly under-damp
            double ki = 0.05;
            w_dot_des = kp * error + ki * I_error + kd * (w_des - w);

            //calculate desired acceleration with PD-control law
            Eigen::Matrix<double, 7, 7> Kp = k_gains_.asDiagonal();
            Eigen::Matrix<double, 7, 7> Kd = d_gains_.asDiagonal();

            Eigen::Matrix<double, 7, 1> tau_measured = Eigen::Map<Eigen::Matrix<double, 7 , 1>>(robot_state.tau_J.data());
            //Eigen::Matrix<double, 7, 1> delta_tau = tau_des - tau_measured - g - tau_bias;
            Eigen::Matrix<double, 6, 1> F_ext = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data());
            delta_F = F_contact_des + F_ext - F_bias; //F_ext shows in the negative direction of F_contact desired
            error_F += dt * delta_F;
            F_last = F_ext;
            //ROS_INFO_STREAM("external estimated Force: " << F_ext);
            //add a task in nullspace. For operational space torque control we construct a Matrix N, which maps a torque tau_n to the nullspace of
            //compute nullspace projection matrix
            Eigen::Matrix<double, 7, 7> N = Eigen::MatrixXd::Identity(7, 7) - J_T * pinv_J_T;
            //set second task torque. In this case joint space control to achieve q_dot = 0
            //dq_desired = pinv_J * w_des; //should least-square minimize q_dot
            Eigen::Matrix<double, 7, 1> ddq = 0.1 * Kp * (q_desired - q) + 0.4 * Kd *(dq_desired - q_dot);
            Eigen::Matrix<double, 7, 1> tau_nullspace = M * ddq;


            //update torque signal with second task

            Eigen::Matrix<double, 6, 1> F_cmd = F_contact_des + 100 * delta_F + 8 * error_F;
            tau_des = J_T * (Lambda * Sm * w_dot_des + Sf * F_cmd + mu); // no need for g because franka compensates it automatically
            tau_des = tau_des +  N * tau_nullspace * additional_task;
            //saturate torque rate and set commands
            if(additional_task == 0){
                std::ofstream stream;
                stream.open("/home/lucas/Desktop/F.txt",std::ios::app);
                if (stream.is_open() && (count < 500)){
                    stream << count << " " << (robot_state.O_F_ext_hat_K) << "\n";
                }
                stream.close();
                count += 1;
                translation_desired += dt * w_des.head(0);
                if ((position(1)>= 0.3) && (direction_changed==false)){
                    w_des = -w_des;
                    direction_changed = true;
                }
                else if ((position(1) <= -0.3) && (direction_changed == true)){
                    direction_changed = false;
                    w_des = -w_des;
                }
            }

            for (size_t i = 0; i < 7; ++i) {
                tau_des = saturateTorqueRate(tau_des, tau_measured);
                joint_handles_[i].setCommand(std::min(tau_des(i), 87.0));
            }


            auto stop = std::chrono::high_resolution_clock::now();
            auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            //std::cout << "Time taken by update: "
             //<< time.count() << " microseconds" << std::endl;
            auto cinStop = std::chrono::high_resolution_clock::now();
        }

    }

    Eigen::Matrix<double, 7, 1> ForceActionController::saturateTorqueRate(
            const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
            const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
        }
        return tau_d_saturated;
    }

} //namespace Force control

PLUGINLIB_EXPORT_CLASS(force_control::ForceActionController, controller_interface::ControllerBase)