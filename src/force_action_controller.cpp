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
        ros::Rate rate = 100;
        if (goal->trajectory.joint_names[0] == "0"){
            additional_task = 0.0; //don't worry about Nullspace torques and set position gain to 0
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
                orientation_desired = orientation;
                started_force_action = false;
            }

            translation_desired(0,0) = goal->trajectory.points[0].positions[0]; //set x-direction
            translation_desired(1,0)  = goal->trajectory.points[0].positions[1]; //set y-direction
            translation_desired(2,0) = goal->trajectory.points[0].positions[2]; //set z-direction

            pose_desired = Eigen::Affine3d::Identity();
            pose_desired.rotate(orientation_desired);
            pose_desired.translation() = translation_desired;


            Eigen::Matrix<double, 6, 1> force_directions;
            force_directions << 0, 0, 1, 0 ,0, 0;
            Sf = force_directions.asDiagonal();
            Sm = IDENTITY - Sf;
            //take guess at desired steady state force

        }
        else{
            started_force_action = true;
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
                double q_tol = 0.0005;
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
        F.open("/home/lucas/Desktop/MA/Force_Data/F_corrections.txt");
        //F << "time,Fx,Fy,Fz,Mx,My,Mz,w_dot_des\n";
        F << "time dF1 dF2 dF3 dF4 dF5 dF6 Fref Fcmd kalman_estimate Fx Fy Fz Mx My Mz\n";
        F.close();

        std::ofstream pose_error;
        pose_error.open("/home/lucas/Desktop/MA/Force_Data/pose_error.txt");
        pose_error << "time x y z dx dy dz\n";
        pose_error.close();

        std::ofstream dtau;
        dtau.open("/home/lucas/Desktop/MA/Force_Data/dtau.txt");
        dtau << "time dtau1 dtau2 dtau3 dtau4 dtau5 dtau6 dtau7 dtau_abs\n";
        dtau.close();

        std::ofstream F_error;
        F_error.open("/home/lucas/Desktop/MA/Force_Data/force_error.txt");
        F_error << "time eFx eFy eFz eMx eMy eMz\n";
        F_error.close();

        std::ofstream w;
        w.open("/home/lucas/Desktop/MA/Force_Data/w_forcing.txt");
        w << "time wx wy wz dox doy doz wref\n";
        w.close();

        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        pose_desired =(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        // set equilibrium point to current state
        translation_desired = pose_desired.translation();
        orientation_desired = Eigen::Quaterniond(pose_desired.rotation());
        // set nullspace equilibrium configuration to initial q
        q_desired = q_initial;
        dq_desired << 0, 0, 0, 0, 0, 0, 0;
        kalman_ext_force = kalman_filter.predict(0);

    }

    void ForceActionController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

            //auto start = std::chrono::high_resolution_clock ::now();
            //if (count == 1){
                //std::cout << "Thread MAIN ID: " << std::this_thread::get_id() << std::endl;
            //}

            update_state();
            Eigen::Matrix<double, 6, 1> error = get_pose_error();
            get_dynamic_model_parameters();
            compute_task_space_matrices(),
            //get current EE-velocity
            w = J * q_dot;
            I_error += dt * error;
            //compute PD-control laq acceleration in operational space
            Eigen::Matrix<double, 6, 6> kp = 600 * IDENTITY;
            double kd = 2*std::sqrt(kp(1,1)) * 1.3; //select kd to slightly over-damp
            double ki = 0.001;
            //calculate desired acceleration with PD-control law
            w_dot_des = kp * error + ki * I_error + kd * (w_des - w);


            Eigen::Matrix<double, 6, 1> F_ext = 0.99 * Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state_.O_F_ext_hat_K.data()) + F_last * 0.01;
            kalman_ext_force = kalman_filter.correct(F_ext(2,1));
            double sign = F_contact_des.dot(F_ext);
           // if(sign > 0){
               // F_ext << 0, 0, 0, 0, 0, 0;
            //}
            F_last = F_ext;
            delta_F = 0.1*(F_contact_des + F_ext) + 0.9 * delta_F; //F_ext shows in the negative direction of F_contact desired
            error_F += dt * delta_F;

            Eigen::Matrix<double, 6, 1> dF_error = (delta_F - delta_F_last) / dt * 0.001 + dF_last * 0.999; //filtered Force error derivative estimate
            dF_last = dF_error;
            delta_F_last = delta_F;

            // 35 , 1 , 5
            F_cmd =  1.5 * delta_F + 1.5 * error_F + 0.1 * dF_error;
            F_cmd *= (1-additional_task);
            kalman_ext_force = kalman_filter.predict(F_cmd(2,1));
            double command_sign = F_cmd.dot(F_contact_des);

            //if (command_sign < 0){
              //  F_cmd << 0, 0, 30, 0, 0, 0;
            //}

            //F_cmd(2, 1) += F_contact_des(2, 1) - 0 * (-9.81 * attached_mass); //feedforward
            Eigen::Matrix<double, 7, 7> Kp = k_gains_.asDiagonal();
            Eigen::Matrix<double, 7, 7> Kd = d_gains_.asDiagonal();
            Eigen::Matrix<double, 7, 1> tau_nullspace = add_nullspace_torque(Kp, Kd);

            tau_des = J_T * (Lambda * Sm * w_dot_des + Sf * F_cmd + mu); // no need for g because franka compensates it automatically
            Eigen::Matrix<double, 6, 1> F_correction = -0.000000003*(1 * delta_F + 0.005 * error_F + 0.1 * dF_error); //unconstrained motion: just compensate external force

            //update torque signal with second task
            tau_des = tau_des + tau_nullspace * additional_task + J_T * Sm * F_correction * (1-additional_task);//Sm only maps force corrections where we move
            //compensate any movement in positive force direction
            Eigen::Matrix<double, 7, 1> tau_measured = Eigen::Map<Eigen::Matrix<double, 7 , 1>>(robot_state_.tau_J_d.data());

            for (size_t i = 0; i < 7; ++i) {
                tau_des = saturateTorqueRate(tau_des, tau_measured);
                joint_handles_[i].setCommand(std::min(tau_des(i), 87.0));
            }

            if(!additional_task && log_rate_()){
                check_movement_and_log(F_correction.transpose(), F_cmd(2,0));
                log_velocity_and_orientation(w, error);

                std::ofstream pose;
                pose.open("/home/lucas/Desktop/MA/Force_Data/pose_error.txt",std::ios::app);
                if (pose.is_open()){
                    pose << count << "," << position.transpose() << "," << translation_desired.transpose() << "\n";
                }
                pose.close();

                std::ofstream dtau_log;
                dtau_log.open("/home/lucas/Desktop/MA/Force_Data/dtau.txt", std::ios::app);
                if (dtau_log.is_open()){
                    dtau_log << count << "," << dq_filtered.transpose() << "," << dtau.norm() << "\n";
                }
                dtau_log.close();

                std::ofstream F_error;
                F_error.open("/home/lucas/Desktop/MA/Force_Data/force_error.txt", std::ios::app);
                if (F_error.is_open()){
                    F_error << count << "," << dF_error.transpose()  << "\n";
                }
                F_error.close();

                count += 1;
            }


            //auto stop = std::chrono::high_resolution_clock::now();
            //auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            //std::cout << "Time taken by update: "
            // << time.count() << " microseconds" << std::endl;

    }


} //namespace Force control

PLUGINLIB_EXPORT_CLASS(force_control::ForceActionController, controller_interface::ControllerBase)