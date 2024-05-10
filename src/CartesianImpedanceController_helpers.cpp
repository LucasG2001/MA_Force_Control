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
            std::ofstream /*testing, F_log, F_error,*/ pose_error, friction_of,  tau_log, optimization  , mass /*, dq_log , coriolis_of, threshold*/;

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

            optimization.open("/home/viktor/Documents/BA/log/optimization.txt", std::ios::app);
            if (optimization.is_open()){
            optimization << count << "," << f.transpose() << " , " << " , " << g.transpose() << " ,  " << dq_imp.transpose() << " , " << z.transpose() << "\n";
            }
            optimization.close();

            // threshold.open("/home/viktor/Documents/BA/log/threshold.txt", std::ios::app);
            // if (threshold.is_open()){
            // threshold << count << "," << q.transpose() << "\n";
            // }
            // threshold.close();   

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
                pose_error << count << "," << error.transpose() << "," << position_d_.transpose() << ","  << desired_orientation.transpose() << "," << tau_friction.transpose() << "\n";
            }
            pose_error.close();

            // dq_log.open("/home/viktor/Documents/BA/log/dq.txt", std::ios::app);
            // if (dq_log.is_open()){
            // dq_log << count << "," << dq.transpose() << " , " << dq_d.transpose() <<  "\n";
            // }
            // dq_log.close();

            // if( count % 100 == 0){
            //     std::cout << (z.array().sign() - dq_imp.array().sign()).transpose() << "\n";
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

    //Calculates the friction forces acting on the robot's joints depending on joint rotational speed. 
    //Exerts torque up to a certain empirically detected static friction threshold. 
    //TODO: Afterwards, goes into the viscous domain and follows a linear raise depending on empiric parameters
    void CartesianImpedanceController::calculate_tau_friction(){
        double alpha = 0.01;//constant for exponential filter in relation to static friction moment        
        dq_filtered = alpha* dq + (1 - alpha) * dq_filtered; //Filtering dq of every joint
        tau_impedance_filtered = alpha*tau_impedance + (1 - alpha) * tau_impedance_filtered; //Filtering tau_impedance
        //Creating and filtering a "fake" tau_impedance with own weights, optimized for friction compensation (else friction compensation would get stronger with higher stiffnesses)
        tau_friction_impedance = jacobian.transpose() * Sm * (-alpha * (D_friction*(jacobian*dq) + K_friction * error)) + (1 - alpha) * tau_friction_impedance;
        //Creating "fake" dq, that acts only in the impedance-space, else dq in the nullspace also gets compensated, which we do not want due to null-space movement
        dq_imp = dq_filtered - N * dq_filtered;

        //Calculation of friction force according to Bachelor Thesis: https://polybox.ethz.ch/index.php/s/iYj8ALPijKTAC2z?path=%2FFriction%20compensation
        f = beta.cwiseProduct(dq_imp) + offset_friction;
        dz = dq_imp.array() - dq_imp.array().abs() / g.array() * sigma_0.array() * z.array() + 0.025* tau_friction_impedance.array()/*(jacobian.transpose() * K * error).array()*/;
        dz(6) -= 0.02*tau_friction_impedance(6);
        z = 0.001 * dz + z;
        tau_friction = sigma_0.array() * z.array() + 100 * sigma_1.array() * dz.array() + f.array();  
    }


}


