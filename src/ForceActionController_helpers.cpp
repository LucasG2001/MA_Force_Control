//
// Created by lucas on 09.05.23.
//
#include <force_control/force_action_controller.h>
#include <chrono>
#include <fstream>
#include <functional>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

namespace force_control{

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
    KalmanFilter::KalmanFilter(double transition_matrix, double input_map, double observer_matrix, double sigma, double epsilon, double x_init) {
        F = transition_matrix;
        B = input_map;
        H = observer_matrix;
        Q = sigma*sigma;
        R = epsilon * epsilon;
        P = Q;
        x = x_init;
    }

    double KalmanFilter::correct(double measurement, const Eigen::Matrix<double, 6, 6>& Lambda, const Eigen::Matrix<double, 6, 1>& w_dot){
        z = measurement;
        double K = P*H/(H*P*H + R);
        x =  x + K*(z-x);
        P = (1 - K*H) * P;
        return x;
    }

    double KalmanFilter::predict(double input, double timestep){
        x = F * x + B * input;
        P = F*P*F + Q;
        return x;
    }

    void ForceActionController::update_state() {
        robot_state_ = franka_state_handle_->getRobotState();
        q_dot = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state_.dq.data ());
        //get current EE-velocity
        w = J * q_dot;
        q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state_.q.data());
        dtau = 0.01 * Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state_.dtau_J.data()) + 0.99 * dtau;
        current_pose = Eigen::Matrix4d::Map(robot_state_.O_T_EE.data());
        position = current_pose.translation();
        orientation =current_pose.rotation();

    }

    void ForceActionController::update_forces() {
        //observed force
        beta_observer = b + g - M_dot * q_dot;
        gamma_observer += ((tau_des + g) - beta_observer + r_observer) * dt;
        r_observer = K_observer * (M*q_dot - gamma_observer);
        tau_observed = r_observer + J_T * F_cmd ;
    }

    void ForceActionController::update_observer() {
        F_ext = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state_.O_F_ext_hat_K.data()) * 0.01 + 0.99 * F_last;
        F_last = F_ext;
        delta_F =  (F_contact_des + F_ext); //F_ext shows in the negative direction of F_contact desired
        error_F += dt * delta_F;
        Eigen::Matrix<double, 6, 1> dF_error = (delta_F - delta_F_last) / dt * 0.1 + dF_last * 0.9; //filtered Force error derivative estimate
        dF_last = dF_error;
        delta_F_last = delta_F;
    }


    Eigen::Matrix<double, 6, 1> ForceActionController::get_pose_error(){
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
        error.tail(3) << current_pose.rotation() * error.tail(3);
        return error;
    }

    void ForceActionController::get_dynamic_model_parameters(){
        //get dynamic model parameters
        std::array<double, 49> mass = model_handle_->getMass();
        std::array<double, 7> coriolis = model_handle_->getCoriolis();
        std::array<double, 7> gravity = model_handle_->getGravity();
        std::array<double, 42> ee_zero_Jac =
                model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

        // set dynamic model parameters into Eigen::Matrices
        M_dot = (Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass.data()) - M)/dt;
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
        force_control::pseudoInverse(J, pinv_J);
    }

    void ForceActionController::compute_task_space_matrices() {
        Lambda = (J * M_inv * J_T).inverse();
        mu = Lambda * (J * M_inv * b - dJ * q_dot);
        p = Lambda * J * M_inv * g;
    }

    Eigen::Matrix<double, 7, 1> ForceActionController::add_nullspace_torque(Eigen::Matrix<double, 7, 7> Kp, Eigen::Matrix<double, 7, 7> Kd){
        //add a task in nullspace. For operational space torque control we construct a Matrix N, which maps a torque tau_n to the nullspace of
        //compute generalized nullspace projection matrix
        Eigen::Matrix<double, 6, 7> ginv_J_T = (J*M_inv*J_T).inverse()*J*M_inv;
        Eigen::Matrix<double, 7, 7> N = Eigen::MatrixXd::Identity(7, 7) - J_T * ginv_J_T;
        //set second task torque
        //dq_desired = pinv_J * w_des; //should least-square minimize q_dot
        Eigen::Matrix<double, 7, 1> ddq =  0.7 * Kp * (q_desired - q) + 1 * Kd * (- q_dot); //set velocity to 0 or dq_des
        Eigen::Matrix<double, 7, 1> tau_nullspace = M * ddq;

        return N * tau_nullspace;
    }

    void ForceActionController::check_movement_and_log(const Eigen::Matrix<double, 1, 6>& data, double commanded_force) {
            std::ofstream stream;
            stream.open("/home/lucas/Desktop/MA/Force_Data/F_corrections.txt",std::ios::app);
            if (stream.is_open()){
                stream << count << "," << data << "," << -F_contact_des(2) << "," <<  commanded_force << "," << kalman_ext_force << "," << robot_state_.O_F_ext_hat_K << "\n";
            }
            stream.close();
    }

    void ForceActionController::log_velocity_and_orientation(const Eigen::MatrixXd& w, const Eigen::MatrixXd& error){
        std::ofstream stream;
        stream.open("/home/lucas/Desktop/MA/Force_Data/w_forcing.txt",std::ios::app);
        if (stream.is_open()){
            stream << count << "," << orientation_desired.x() << "," << orientation_desired.y() << "," << orientation_desired.z() << "," <<
            orientation.x() << "," << orientation.y() << "," << orientation.z() << "," << w_des(1) <<"\n";
        }
        //ROS_INFO_STREAM("logged pose error is = " << error.block(3, 0, 3, 1).transpose());
        stream.close();
    }

    //saturates torque rate at 1000 Nm/s (setting kdeltaTauMax accordingly as dt*1000)
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

    // updates friction torque estimates according to model from friction parameters taken from https://inria.hal.science/hal-02265294/document
    void ForceActionController::update_friction_torque() {
        for(int i =4; i<7; i++){


            double a = phi1[i];
            double b = phi2[i];
            double c = phi3[i];
            double term1 = 1.0 + exp(-b * (q_dot(i,0) + c));
            double term2 = 1.0 + exp(-b * c);


            double dynamic_friction = 1*((a/term1) -(a/term2));

            //double v_sign = q_dot(i,0)/std::abs(q_dot(i,0));
            int static_sign;
            if ((J_T*Lambda*w_dot_des)(i,0) > 0.001){
                static_sign=1;
            }
            else if((J_T*Lambda*w_dot_des)(i,0) < -0.001){
                static_sign = -1;
            }
            else{
                static_sign = 0;
            }
            //double sign = tanh(1000*q_dot(i,0));
            //double dynamic_friction = fv[i] * q_dot(i,0) + fc[i] * q_dot(i,0) + 0*fo[i];
            //dynamic friction (filtered)
            dynamic_friction_torques(i, 0) =
                   dynamic_friction;
            //static friction
            friction_torques(i,0) = 0.001*
                    (static_sign * static_friction_torque[i] * exp(-std::abs(q_dot(i,0))) + dynamic_friction_torques(i,0))
                    + 0.999 * friction_torques(i,0);

            /**
            if(i==6){
                ROS_INFO_STREAM("sign is " << static_sign);
                ROS_INFO_STREAM("static torque is " <<  static_sign * static_friction_torque[i] * exp(-100*std::abs(q_dot(i,0))));
                ROS_INFO_STREAM("dynamic torque is " <<  dynamic_friction_torques(i,0));
            }
            **/

        }
    }

} //namespace force control
