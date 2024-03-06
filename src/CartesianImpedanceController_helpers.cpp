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
            std::ofstream testing/*, F_log, F_error*/, pose_error, friction_of,  tau_log, optimization  , mass/*, dq_log , coriolis_of, threshold*/;

            // testing.open("/home/viktor/Documents/BA/log/testing.txt", std::ios::app);
            // if (testing.is_open()){
            //     testing << count << " , " << joint << " , " << sigma_0_guess << " , " << q(joint) << " , " << tau_d(joint) << " , " << z_guess << " , " << x_integral << "\n";
            // }
            // testing.close();

            tau_log.open("/home/viktor/Documents/BA/log/tau.txt", std::ios::app);
            if (tau_log.is_open()){
            tau_log << count << "," << tau_d.transpose() << "\n";
            }
            tau_log.close();  

            mass.open("/home/viktor/Documents/BA/log/mass.txt", std::ios::app);
            if (mass.is_open()){
                mass << M << "\n" << "\n";
            }
            mass.close();


            // coriolis_of.open("/home/viktor/Documents/BA/log/coriolis_of.txt", std::ios::app);
            // if (coriolis_of.is_open()){
            // coriolis_of << count << "," << coriolis.transpose() << " , " << tau_J_d.transpose() << "\n";
            // }
            // coriolis_of.close();   

            friction_of.open("/home/viktor/Documents/BA/log/friction_of.txt", std::ios::app);
            if (friction_of.is_open()){
            friction_of << count << "," << -r.transpose() << " , " << friction_optimized.transpose() << " , " << tau_friction.transpose() << "\n";
            }
            friction_of.close();

            optimization.open("/home/viktor/Documents/BA/log/optimization.txt", std::ios::app);
            if (optimization.is_open()){
            optimization << count << "," << f.transpose() << " , " << " , " << g.transpose() << " ,  " << dq_imp.transpose() << " , " << q.transpose() << "\n";
            }
            optimization.close();

            // threshold.open("/home/viktor/Documents/BA/log/threshold.txt", std::ios::app);
            // if (threshold.is_open()){
            // threshold << count << "," << tau_threshold_min.transpose() << "\n";
            // }
            // threshold.close();   

            // F_log.open("/home/viktor/Documents/BA/log/F.txt", std::ios::app);
            // if (F_log.is_open()){
            //     F_log << count << "," << F_friction_keep.transpose() << "\n";
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
                pose_error << count << "," << error.transpose() << "," << position_d_.transpose() << ","  << desired_orientation.transpose() << "," << -error_threshold.transpose() << "\n";
            }
            pose_error.close();

            // dq_log.open("/home/viktor/Documents/BA/log/dq.txt", std::ios::app);
            // if (dq_log.is_open()){
            // dq_log << count << "," << dq.transpose() << " , " << dq_d.transpose() <<  "\n";
            // }
            // dq_log.close();

            // if( count % 100 == 0){
            //     std::cout << N << "\n";
            //     std::cout << "---------------------------------------------------- \n";
            // }

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

    int sgn(double number){ //returns sign_tau of number
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
        g = coulomb_friction;
        file.close(); // Close the file after reading
        static_friction_minus = offset_friction - coulomb_friction;

    }


    //Calculates the friction forces acting on the robot's joints depending on joint rotational speed. 
    //Exerts torque up to a certain empirically detected static friction threshold. 
    //TODO: Afterwards, goes into the viscous domain and follows a linear raise depending on empiric parameters
    void CartesianImpedanceController::calculate_tau_friction(){
        
        double alpha = 0.01;//constant for exponential filter in relation to static friction moment
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
            int sign_tau = sgn(tau_impedance_filtered(i));
            int sign_dq = sgn(dq_filtered(i));  
            double dq_filtered_abs = std::abs(dq_filtered(i));

            // if (dq_filtered_abs < 0.005 || i == 1){
            //     //tau_friction(i) = 2 * coulomb_friction(i) / (1 + std::exp(sigmoid_param(i)*std::abs(dq_filtered(i))* sign_tau)) + static_friction_minus(i); //sigmoid function for region around 0
            //     tau_friction(i) = sign_tau * coulomb_friction(i) + offset_friction(i);
            //     friction_state(i) = 1;
            // }//static friction, friction is constant for joint 1 at every speed

            // else if( i == 6){
            //     tau_friction(i) = (qua_a(6) + (qua_b(6) - qua_a(6)) * exp(-1 * std::abs(dq_filtered(6)/qua_c(6)))) * sign_dq + lin_a(6) * dq_filtered(i);
            // }

            // else if (dq_filtered_abs >= friction_region_change(i)){
            //     tau_friction(i) = lin_a(i)*sign_dq + lin_b(i)*dq_filtered(i) + offset_friction(i);
            //     friction_state(i) = 3;
            // }
            // else{
            //     tau_friction(i) = qua_a(i)*sign_dq + qua_b(i)*dq_filtered(i) + qua_c(i) * dq_filtered(i)*dq_filtered(i) *sign_dq + offset_friction(i);
            //     friction_state(i) = 2;
            // }

        }

        state_tuner();
        // Eigen::VectorXd tau_friction_nullspace (7);
        // tau_friction_nullspace = (Eigen::MatrixXd::Identity(7, 7) -
        //                   jacobian.transpose() * jacobian_transpose_pinv) * tau_friction;
        // tau_friction -= tau_friction_nullspace;


        // Eigen::VectorXd F_friction_theory (6), K_vector(6);
        // F_friction_theory = jacobian_transpose_pinv * -1 * r;
        // K_vector << K.diagonal();
        // error_threshold << F_friction_theory.cwiseQuotient(K_vector);
        // for(int i = 0; i < 6; ++i){
        //     //F_friction_keep(i) = F_friction_theory(i);
        //     F_friction_keep(i) = std::min(std::abs(error(i))/error_goal(i), 1.0) * F_friction_theory(i);
        // }
        tau_friction = friction_optimized;
        
    }


    void CartesianImpedanceController::state_observer(){
        Eigen::Matrix<double, 7, 7> dM = (M - M_old).array()/0.001; 
        integral_observer += (tau_d + dM*dq - coriolis+r)*0.001;
        r = K_0 * (M*dq - integral_observer);
        r[6] *= 0;
        M_old = M;
    }

    void CartesianImpedanceController::state_tuner(){
        dq_imp = dq - N * dq_filtered;
        g(6) = (qua_a(6) + (qua_b(6) - qua_a(6)) * exp(-1 * std::abs(dq_imp(6)/qua_c(6)))) * sgn(dq_imp(6)) + lin_a(6) * dq_imp(6);
        f = lin_b.cwiseProduct(dq_imp) + offset_friction;
        //sigma_0 = (r - f - dq_filtered).array() / (z.array() - dq_filtered.array().abs() / g.array() * z.array());
        dz = dq_imp.array() - dq_imp.array().abs() / g.array() * sigma_0.array() * z.array();
        z = 0.001 * dz + z;
        friction_optimized = sigma_0.array() * z.array() + sigma_1.array() * dz.array() + f.array();
        // friction_optimized = friction_optimized.array().abs() * tau_impedance_filtered.array().sign();
    }

}


