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
            std::ofstream /*F_log, F_error,*/ pose_error , friction_of,  tau_log /*, dq_log , coriolis_of*/, threshold;

            tau_log.open("/home/viktor/Documents/BA/log/tau.txt", std::ios::app);
            if (tau_log.is_open()){
            tau_log << count << "," << tau_impedance_filtered.transpose() << " , " << gravity.transpose() << "\n";
            }
            tau_log.close();   

            // coriolis_of.open("/home/viktor/Documents/BA/log/coriolis_of.txt", std::ios::app);
            // if (coriolis_of.is_open()){
            // coriolis_of << count << "," << coriolis.transpose() << " , " << tau_J_d.transpose() << "\n";
            // }
            // coriolis_of.close();   

            friction_of.open("/home/viktor/Documents/BA/log/friction_of.txt", std::ios::app);
            if (friction_of.is_open()){
            friction_of << count << "," << tau_friction.transpose() << " , " << friction_state.transpose() << "\n";
            }
            friction_of.close();

            threshold.open("/home/viktor/Documents/BA/log/threshold.txt", std::ios::app);
            if (threshold.is_open()){
            threshold << count << "," << tau_threshold_min.transpose() << "\n";
            }
            threshold.close();   

            // F_log.open("/home/viktor/Documents/BA/log/F.txt", std::ios::app);
            // if (F_log.is_open()){
            //     F_log << count << "," << F_contact_des(2,0) << "," << F_cmd(2,0) << "," << F_ext.transpose() << "," << F_impedance.transpose() << "\n";
            // }
            // F_log.close();

            // F_error.open("/home/lucas/Desktop/MA/Force_Data/friction_of.txt", std::ios::app);
            // if (F_error.is_open()){
            //     F_error << count << "," << I_error.transpose() << "," << 0 << "," << "\n";
            // }
            // F_error.close();

            Eigen::Vector3d desired_orientation = orientation_d_.toRotationMatrix().eulerAngles(0,1,2);
            pose_error.open("/home/viktor/Documents/BA/log/pose_error.txt", std::ios::app);
            if (pose_error.is_open()){
                pose_error << count << "," << error.transpose() << "," << position_d_.transpose() << ","  << desired_orientation.transpose() << "," << "\n";
            }
            pose_error.close();

            // dq_log.open("/home/viktor/Documents/BA/log/dq.txt", std::ios::app);
            // if (dq_log.is_open()){
            // dq_log << count << "," << dq.transpose() << " , " << dq_d.transpose() <<  "\n";
            // }
            // dq_log.close();

            if( count % 10 == 0){
                std::cout << tau_threshold_separate << " , " << error_goal_met << "\n";
                std::cout << "---------------------------------------------------- \n";
            }

            count += 1;
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


   //Loads friction parameters into vector friction_parameters from file specified in filePath
    void CartesianImpedanceController::load_friction_parameters(const std::string& filePath){
        Eigen::VectorXd friction_parameters(49);
        std::ifstream file(filePath);
        
        if (!file.is_open()) {
            std::cerr << "Error opening file!" << std::endl;
        }

        std::string line;
        int i = 0; 

        while (i < 49 && std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                // Disregard empty lines or lines starting with '#'
                continue;
            }

            // Attempt to convert the line to a double
            std::istringstream iss(line);
            double value;
            if (!(iss >> friction_parameters(i))) {
                std::cerr << "Failed to convert line to double: " << line << " Entry: " << i << std::endl;
                // Handle the error or continue to the next line
                continue;
            }
            ++i;

        }

        coulomb_friction = friction_parameters.segment(0, 7);
        offset_friction = friction_parameters.segment(7,7);
        lin_a = friction_parameters.segment(14,7);
        lin_b = friction_parameters.segment(21,7);
        qua_a = friction_parameters.segment(28,7);
        qua_b = friction_parameters.segment(35,7);
        qua_c = friction_parameters.segment(42,7);
        file.close(); // Close the file after reading
        static_friction_minus = offset_friction - coulomb_friction;

    }


    void CartesianImpedanceController::linear_friction(const int &i){
        tau_friction(i) = lin_a(i)*sgn(dq_filtered(i)) + lin_b(i)*dq_filtered(i) + offset_friction(i);
    }

    void CartesianImpedanceController::quadratic_friction(const int &i){
        tau_friction(i) = qua_a(i)*sgn(dq_filtered(i)) + qua_b(i)*dq_filtered(i) + qua_c(i) * dq_filtered(i)*dq_filtered(i) *sgn(dq_filtered(i)) + offset_friction(i);
    }

    //Calculates the friction forces acting on the robot's joints depending on joint rotational speed. 
    //Exerts torque up to a certain empirically detected static friction threshold. 
    //TODO: Afterwards, goes into the viscous domain and follows a linear raise depending on empiric parameters
    void CartesianImpedanceController::calculate_tau_friction(){
        
        double alpha = 0.1;//constant for exponential filter in relation to static friction moment
        tau_threshold = jacobian.transpose() * Sm * K * error_goal;//minimal moment to achieve target precision by stiffness
        tau_threshold_separate = jacobian.transpose() * Sm * K * error_goal_separate;
        
        Eigen::VectorXd error_goal_met_double(6);
        error_goal_met_double = error_goal_met.cast<double>();
        tau_threshold_separate.array().rowwise() *= (1-error_goal_met_double.array().transpose()); 

        //After which speed we go from quadratic to linear friction model:
        Eigen::VectorXd friction_region_change = (Eigen::VectorXd(7)  << 0.005, 0.005, 0.005, 0.005, 0.2, 0.005, 0.15).finished();
        bool turn_on = false; //Tells us, whether friction compensation should be on depending on the tau_impedance values we get from impedance control



        // for (int i = 0; i < 7; ++i){ 
        //     if (std::abs(tau_impedance_filtered(i)) > std::abs(tau_threshold(i))){
        //         turn_on = true;
        //         break;
        //     }
        // }//If at least one of the joints is experiencing tau_impedance, all the joints should do friction compensation

        for(int i = 0; i < 7; ++i){
                
            tau_threshold_min(i) = tau_threshold_separate.row(i).sum();
            dq_filtered(i) = alpha * dq(i) + (1 - alpha)*dq_filtered(i);
            tau_impedance_filtered(i) = alpha*tau_impedance(i) + (1 - alpha)*tau_impedance_filtered(i);

            if (/*!turn_on*/0.05 > std::abs(tau_threshold_min(i)) || sgn(tau_impedance_filtered(i)) != sgn(tau_friction(i))){
                tau_friction(i) *= 1-alpha; 
                friction_state(i) = 0;
            }//If tau_impedance is below tau_threshold, we are already accurate enough, no more movement and thus no friction compensation is needed

            else if (std::abs(dq_filtered(i)) < 0.005 || i == 1){
                tau_friction(i) = 2 * coulomb_friction(i) / (1 + std::exp(-600*std::abs(dq_filtered(i))*sgn(tau_impedance_filtered(i)))) + static_friction_minus(i); //sigmoid function for region around 0
                friction_state(i) = 1;
            }//static friction, friction is constant for joint 1 at every speed

            else if (std::abs(dq_filtered(i)) >= friction_region_change(i)){
                linear_friction(i);
                friction_state(i) = 3;
            }
            else{
                quadratic_friction(i);
                friction_state(i) = 2;
            }
            tau_friction(i) = sgn(tau_impedance_filtered(i))*std::abs(tau_friction(i));

        }


    }

}


