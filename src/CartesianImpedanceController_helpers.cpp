#include <force_control/cartesian_impedance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <chrono>
#include <functional>

namespace force_control {


    void CartesianImpedanceController::update_stiffness_and_references(){
        // update parameters changed online either through dynamic reconfigure or through the interactive
        // target by filtering
        /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
        K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
        D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
        nullspace_stiffness_ =
                filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
        std::lock_guard<std::mutex> position_d_target_mutex_lock(
                position_and_orientation_d_target_mutex_);
        position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
        orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

    }

    void CartesianImpedanceController::log_values_to_file(bool log){
        Eigen::Matrix<double, 6, 1> integrator_weights;
        if(log){
            std::ofstream /*F_log, F_error, pose_error,*/ tau_log;
            tau_log.open("/home/viktor/Documents/BA/log/tau.txt", std::ios::app);
            //std::cout << "test";
            if (tau_log.is_open()){
            tau_log << count << "," << tau_measured.transpose() << " velocities " << vel_measured.transpose() << "\n";
            }

            tau_log.close();       
            // F_log.open("/home/lucas/Desktop/MA/Force_Data/F_corrections.txt", std::ios::app);
            // if (F_log.is_open()){
            //     F_log << count << "," << F_contact_des(2,0) << "," << F_cmd(2,0) << "," << F_ext.transpose() << "," << F_impedance.transpose() << "\n";
            // }

            // F_log.close();
            // F_error.open("/home/lucas/Desktop/MA/Force_Data/friction.txt", std::ios::app);
            // if (F_error.is_open()){
            //     F_error << count << "," << I_error.transpose() << "," << 0 << "," << "\n";
            // }
            // F_error.close();
            // Eigen::Vector3d desired_orientation = orientation_d_.toRotationMatrix().eulerAngles(0,1,2);
            // pose_error.open("/home/lucas/Desktop/MA/Force_Data/pose_error.txt", std::ios::app);
            // if (pose_error.is_open()){
            //     pose_error << count << "," << error.transpose() << "," << position_d_.transpose() << ","  << desired_orientation.transpose() << "," << "\n";
            // }
            // pose_error.close();

            count += 1;
        }
    }

    Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
            const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
            const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] =
                    tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }
        return tau_d_saturated;
    }

    int sgn(double number){ //returns sign of number
        if(number >= 0){
            return 1;
        }
        return -1;
    }

    Eigen::VectorXd CartesianImpedanceController::calculate_tau_friction(const Eigen::Matrix<double, 7, 1> dq){
        Eigen::VectorXd F_c(7), beta(7), tau_friction(7); //Initialize basic Coulomb's friction model
        for (int i = 0; i < 7; i++){
            tau_friction(i) = F_c(i) * sgn(dq(i)) + beta(i) * dq(i); //Calculate resulting friction torque for every joint

        }
            
        return tau_friction;
    }

}
