//
// Created by lucas on 09.05.23.
//
#include <force_control/force_action_controller.h>
#include <pluginlib/class_list_macros.h>
#include <chrono>
#include <fstream>
#include <functional>
#include <thread>

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

    void ForceActionController::update_state() {
        robot_state_ = franka_state_handle_->getRobotState();
        q_dot = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state_.dq.data ());
        q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state_.q.data());
        current_pose = Eigen::Matrix4d::Map(robot_state_.O_T_EE.data());
        position = current_pose.translation();
        orientation =current_pose.rotation();

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
        //compute nullspace projection matrix
        Eigen::Matrix<double, 7, 7> N = Eigen::MatrixXd::Identity(7, 7) - J_T * pinv_J_T;
        //set second task torque
        //dq_desired = pinv_J * w_des; //should least-square minimize q_dot
        Eigen::Matrix<double, 7, 1> ddq = 0.1 * Kp * (q_desired - q) + 0.4 * Kd *(dq_desired - q_dot);
        Eigen::Matrix<double, 7, 1> tau_nullspace = M * ddq;

        return N * tau_nullspace;
    }

    void ForceActionController::check_movement_and_log(bool log) {
        if(additional_task == 0){
            if(log) {
                std::ofstream stream;
                stream.open("/home/lucas/Desktop/F.txt",std::ios::app);
                if (stream.is_open()){
                    stream << count << " " << (robot_state_.O_F_ext_hat_K) << "\n";
                }
                stream.close();
            }
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

} //namespace force control
