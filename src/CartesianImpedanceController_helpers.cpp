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
            std::ofstream /*F_log, F_error, pose_error , friction_of, tau_log,*/ dq_log, coriolis_of /*threshold*/;

            // tau_log.open("/home/viktor/Documents/BA/log/tau.txt", std::ios::app);
            // //std::cout << "test";
            // if (tau_log.is_open()){
            // tau_log << count << "," << tau_J_d.transpose() << "\n";
            // }
            // tau_log.close();   

            coriolis_of.open("/home/viktor/Documents/BA/log/coriolis_of.txt", std::ios::app);
            //std::cout << "test";
            if (coriolis_of.is_open()){
            coriolis_of << count << "," << coriolis.transpose() << " , " << tau_J_d.transpose() << "\n";
            }
            coriolis_of.close();   

            // friction_of.open("/home/viktor/Documents/BA/log/friction_of.txt", std::ios::app);
            // if (friction_of.is_open()){
            // friction_of << count << "," << tau_friction.transpose() << " , " << friction << "\n";
            // }
            // friction_of.close();

        //    < threshold.open("/home/viktor/Documents/BA/log/threshold.txt", std::ios::app);
        //     if (threshold.is_open()){
        //     threshold << count << "," << tau_threshold.transpose() << "\n";
        //     }
        //     threshold.close();   >

            // F_log.open("/home/lucas/Desktop/MA/Force_Data/F_corrections.txt", std::ios::app);
            // if (F_log.is_open()){
            //     F_log << count << "," << F_contact_des(2,0) << "," << F_cmd(2,0) << "," << F_ext.transpose() << "," << F_impedance.transpose() << "\n";
            // }
            // F_log.close();

            // F_error.open("/home/lucas/Desktop/MA/Force_Data/friction_of.txt", std::ios::app);
            // if (F_error.is_open()){
            //     F_error << count << "," << I_error.transpose() << "," << 0 << "," << "\n";
            // }
            // F_error.close();

            // Eigen::Vector3d desired_orientation = orientation_d_.toRotationMatrix().eulerAngles(0,1,2);
            // pose_error.open("/home/viktor/Documents/BA/log/pose_error.txt", std::ios::app);
            // if (pose_error.is_open()){
            //     pose_error << count << "," << error.transpose() << "," << position_d_.transpose() << ","  << desired_orientation.transpose() << "," << "\n";
            // }
            // pose_error.close();

            dq_log.open("/home/viktor/Documents/BA/log/dq.txt", std::ios::app);
            //std::cout << "test";
            if (dq_log.is_open()){
            dq_log << count << "," << dq.transpose() << " , " << dq_d.transpose() <<  "\n";
            }
            dq_log.close();

            if( count % 1000 == 0){
                std::cout << M << "\n";
                std::cout << "---------------------------------------------------- \n";
                std::cout << joint;
                std::cout << "---------------------------------------------------- \n";
            }

            count += 1;
        }
    }

    //Loads friction parameters into vector friction_parameters from file specified in filePath
    void CartesianImpedanceController::load_friction_parameters(const std::string& filePath){
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Error opening file!" << std::endl;
        }
        for (int i = 0; i < 7; ++i){
            file >> friction_parameters(i);
        }
    }

    //Saturate TorqueRate to delta_tau_max_ defined in the header file
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


    //Calculates the friction forces acting on the robot's joints depending on joint rotational speed. 
    //Exerts torque up to a certain empirically detected static friction (friction_parameters) threshold. 
    //TODO: Afterwards, goes into the viscous domain and follows a linear raise depending on empiric parameters
    void CartesianImpedanceController::calculate_tau_friction(){
        //static friction
        Eigen::VectorXd stepsize = friction_parameters / 200.0;
        
        //linear filtering

        //Linearly increases until reaches friction_parameters-threshold. Tau_friction always has the sign of tau_impedance
        //Linearly decreases if joint begins moving or when tau_impedance changes signs
        // for (int i = 0; i < 7; ++i){
        //     if (std::abs(dq(i)) < 0.005 && std::abs(tau_impedance(i)) > 0.5){
        //         tau_friction(i) += sgn(tau_impedance(i)) * stepsize(i);
        //         tau_friction(i) = sgn(tau_friction(i)) * std::min(std::abs(tau_friction(i)), friction_parameters(i));
        //     }
        //     else if(tau_friction(i) != 0){
        //         tau_friction(i) = sgn(tau_friction(i)) * std::max(std::abs(tau_friction(i)) - stepsize(i), 0.0 );
        //     }
        // }  

        //exponential moving filter
        Eigen::VectorXd alpha = friction_parameters / 100.0;
        tau_threshold = jacobian.transpose() * Sm * K * error_goal;

        for(int i = 0; i < 7; ++i){
             if (std::abs(dq(i)) < 0.005 && std::abs(tau_impedance(i)) >= std::abs(tau_threshold(i))){
                tau_friction(i) = sgn(tau_impedance(i)) * alpha(i) * friction_parameters(i) + (1 - alpha(i))*tau_friction(i);
             }
                else{
                    tau_friction(i) *= 1-alpha(i); 
                }
        }


    }

}


