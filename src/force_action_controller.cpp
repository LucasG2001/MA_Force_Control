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
        k_gains_ << 600, 600, 600, 600, 250, 150, 200;
        d_gains_ << 30, 30, 30, 30, 10, 10, 5;
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
        F_error.open("/home/lucas/Desktop/MA/Force_Data/friction.txt");
        F_error << "time eFx eFy eFz eMx eMy eMz f7\n";
        F_error.close();

        std::ofstream w;
        w.open("/home/lucas/Desktop/MA/Force_Data/w_forcing.txt");
        w << "time wx wy wz dox doy doz wref\n";
        w.close();

        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        pose_desired =(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d deviation;
        deviation << 0.0, 0.0, 0.0;
        translation_desired = pose_desired.translation() + deviation;
        orientation_desired = Eigen::Quaterniond(pose_desired.rotation());
        translation_target = translation_desired;
        orientation_target = orientation_desired;
        q_desired = q_initial;
        dq_desired << 0, 0, 0, 0, 0, 0, 0;
        kalman_ext_force = kalman_filter.predict(0, dt);


        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(0.0);
        }

    }

    void ForceActionController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
            //auto start = std::chrono::high_resolution_clock ::now();
            //if (count == 1){
            //std::cout << "Thread MAIN ID: " << std::this_thread::get_id() << std::endl;
            //}

            get_dynamic_model_parameters();
            update_state();
            compute_task_space_matrices();
            update_forces();
            update_friction_torque();
            pose_error = get_pose_error();


            //compute PD-control laq acceleration in operational space
            Eigen::Matrix<double, 6, 1> kp_values;
            Eigen::Matrix<double, 6, 1> kd_values;
            Eigen::Matrix<double, 6, 1> ki_values;
            kp_values << 600, 600, 600, 1750, 1750, 2000; //... 400,400,2000
            kd_values << 60, 60, 60, 5, 5, 5;
            ki_values << 5, 5, 5, 150, 150, 500; //...5, 5, 50
            //Eigen::Matrix<double, 6, 6> kp = 40 * IDENTITY;s
            Eigen::Matrix<double, 6, 6> kp = kp_values.asDiagonal();
            Eigen::Matrix<double, 6, 6> kd = kd_values.asDiagonal();
            Eigen::Matrix<double, 6, 6> ki = ki_values.asDiagonal();



            //kalman_ext_force = kalman_filter.correct(F_ext(2,1), Lambda, w_dot_des);

            Eigen::Matrix<double, 7, 7> Kp = k_gains_.asDiagonal();
            Eigen::Matrix<double, 7, 7> Kd = d_gains_.asDiagonal();
            Eigen::Matrix<double, 7, 1> tau_nullspace = add_nullspace_torque(Kp, Kd);

        switch (control_mode) {
            case 0:
                /**
                F_cmd = (1.9 * delta_F + 13.5 * error_F + 0.01 * dF_error) * control_mode;
                if ((J * dq_filtered)(2, 0) > 0.005) {
                    F_cmd = 0.3 * F_contact_des;
                }
                F_cmd *= 0;
                **/
                //calculate desired acceleration with PD-control law
                w_dot_des = kp * pose_error + ki * I_error + kd * (w_des - w);
                for(int i=0; i<6; i++){
                    w_dot_des(i,0) = std::max(-max_accelerations[i] * 0.5, std::min(w_dot_des(i,0), max_accelerations[i] * 0.5));
                }
                tau_des = J_T * (Lambda * Sm * w_dot_des + Sf * F_cmd + mu) +
                         1 * tau_nullspace + friction_torques; // no need for g because franka compensates it automatically
                //cartesian control
                break;
            case 1:
                //joint control
                ddq_desired = Kp * (q_desired - q) + Kd * (dq_desired - q_dot);
                tau_des = M * ddq_desired + b + friction_torques; //g is added internally by libfranka
                break;
            }

            //update force observers
            //kalman_ext_force = kalman_filter.predict(F_cmd(2,1), dt);
            //update_observer();

            tau_des = saturateTorqueRate(tau_des, Eigen::Map<Eigen::Matrix<double, 7 , 1>>(robot_state_.tau_J_d.data()));

            for (size_t i = 0; i < 7; ++i) {
                joint_handles_[i].setCommand(tau_des(i));
            }

            if(control_mode == 0 && log_rate_()){ //log control mode = 0 -> cartesian, control_mode = 1 -> joint state
                check_movement_and_log(F_ext.transpose(), F_cmd(2,0));
                log_velocity_and_orientation(w, pose_error);

                std::ofstream pose;
                pose.open("/home/lucas/Desktop/MA/Force_Data/pose_error.txt",std::ios::app);
                if (pose.is_open()){
                    pose << count << "," << position.transpose() << "," << translation_desired.transpose() << "\n";
                }
                pose.close();
                //tau_observed(1,0) = (pinv_J_T*tau_des)(2,0);
                std::ofstream dtau_log;
                dtau_log.open("/home/lucas/Desktop/MA/Force_Data/dtau.txt", std::ios::app);
                if (dtau_log.is_open()){
                    dtau_log << count << "," << tau_des.transpose() << "," << dtau.norm() << "\n";
                }
                dtau_log.close();

                std::ofstream F_error;
                F_error.open("/home/lucas/Desktop/MA/Force_Data/friction.txt", std::ios::app);
                if (F_error.is_open()){
                    F_error << count << "," << dynamic_friction_torques.transpose() << "," << "\n";
                }
                F_error.close();

                count += 1;
            }


            //auto stop = std::chrono::high_resolution_clock::now();
            //auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            //std::cout << "Time taken by update: "
            // << time.count() << " microseconds" << std::endl;
        translation_desired = 0.1 * translation_target + (1.0 - 0.1) * translation_desired;
        //orientation_desired = orientation_desired.slerp(0.1, orientation_target);
        orientation_desired = orientation_target;

        }


} //namespace Force control

PLUGINLIB_EXPORT_CLASS(force_control::ForceActionController, controller_interface::ControllerBase)