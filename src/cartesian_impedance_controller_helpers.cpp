#include <force_control/cartesian_impedance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <chrono>
#include <functional>

namespace force_control {


    void CartesianImpedanceController::update_stiffness_and_references(){
        // update parameters changed online either through dynamic reconfigure or through the interactive
        // target by filtering
        /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
        // impedance parameters
		//T =  filter_params_ * cartesian_inertia_target_ + (1.0 - filter_params_) * T;
		//at the moment we cannot use variable inertia
        K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
        //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
		//safety bubble
	    repulsion_K = filter_params_ * repulsion_K_target_ + (1.0 - filter_params_) * repulsion_K;
	    repulsion_D = filter_params_ * repulsion_D_target_ + (1.0 - filter_params_) * repulsion_D;
		//nullspace stiffness
        nullspace_stiffness_ =
                filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
        std::lock_guard<std::mutex> position_d_target_mutex_lock(
                position_and_orientation_d_target_mutex_);
		//desired position and force
		//ToDo:: check what happens if we do no interpolation on reference pose
        position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
	    velocity_d_ = filter_params_ * velocity_d_target_ + (1.0 - filter_params_) * velocity_d_;
        orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
		F_contact_des = filter_params_ * F_contact_target + filter_params_ * F_contact_des;

    }

    void CartesianImpedanceController::log_values_to_file(bool log){
		//ToDo: This can be moved to a logging node or at least parts of it
        if(log){
            std::ofstream F_log, F_error, pose_error, hand_tracking, potential_field;


            F_log.open("/home/lucas/Desktop/MA/Force_Data/F_corrections.txt", std::ios::app);
            if (F_log.is_open()){
                F_log << count << "," << F_contact_des(2,0) << "," << F_cmd(2,0) << "," << F_ext.transpose() << "," << F_impedance.transpose() << "\n";
            }
            F_log.close();
			/*
            F_error.open("/home/lucas/Desktop/MA/Force_Data/friction.txt", std::ios::app);
            if (F_error.is_open()){
                F_error << count << "," << I_error.transpose() << "," << 0 << "," << "\n";
            }
			F_error.close();
			 */



			Eigen::Vector3d desired_orientation = orientation_d_target_.toRotationMatrix().eulerAngles(0,1,2);
			pose_error.open("/home/lucas/Desktop/MA/Force_Data/pose_error.txt", std::ios::app);
			if (pose_error.is_open()){
				pose_error << count << "," << error.transpose() << "," << position_d_.transpose() << ","  << desired_orientation.transpose() << "," << "\n";
			}
			pose_error.close();
			/*
			hand_tracking.open("/home/lucas/Desktop/MA/Force_Data/hand_tracking.txt", std::ios::app);
			if (hand_tracking.is_open()){
				hand_tracking << count << "," << C.transpose() << "," << r.transpose() << ","  << F_repulsion.head(3).transpose() << "\n";
			}
			hand_tracking.close();

			potential_field.open("/home/lucas/Desktop/MA/Force_Data/potential_force.txt", std::ios::app);
			if (potential_field.is_open()){
				potential_field << count << "," << F_potential.head(3).transpose() << "\n";
			}
			potential_field.close();
			*/

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


}
