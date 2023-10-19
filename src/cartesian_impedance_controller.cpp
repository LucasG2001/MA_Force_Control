//
// Created by lucas on 16.06.23.
//
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <force_control/cartesian_impedance_controller.h>
#include <cmath>
#include <memory>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <franka_msgs/SetForceTorqueCollisionBehavior.h>
#define IDENTITY Eigen::MatrixXd::Identity(6,6)


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

    bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                                   ros::NodeHandle& node_handle) {


        // free move scenario pose reference
        sub_equilibrium_pose_ = node_handle.subscribe(
                "reference_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //control mode between free float and impedance
        sub_control_mode = node_handle.subscribe(
                "control_mode", 20, &CartesianImpedanceController::control_mode_callback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //subscriber for Yannic Hofmanns and Lucas Gimenos Hololens-Teleoperation code
        sub_eq_config = node_handle.subscribe(
                "joint_angles", 20, &CartesianImpedanceController::JointConfigCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //position of right hand for safety bubble
        sub_hand_pose = node_handle.subscribe(
                "right_hand", 20, &CartesianImpedanceController::HandPoseCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //movement with constrained motion in one direction, where force can be applied
        sub_force_action = node_handle.subscribe(
                "panda_force_action", 1, &CartesianImpedanceController::force_callback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //subscribes to the complete resulting force from planning scene potential field
        sub_potential_field = node_handle.subscribe(
                "/resulting_force", 20, &CartesianImpedanceController::potential_field_callback, this,
                ros::TransportHints().reliable().tcpNoDelay());

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
            return false;
        }
        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
            ROS_ERROR(
                    "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
                    "aborting controller init!");
            return false;
        }

        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM(
                    "CartesianImpedanceController: Error getting model interface from hardware");
            return false;
        }
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                    model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                    "CartesianImpedanceController: Exception getting model handle from interface: "
                            << ex.what());
            return false;
        }

        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM(
                    "CartesianImpedanceController: Error getting state interface from hardware");
            return false;
        }
        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                    state_interface->getHandle(arm_id + "_robot"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                    "CartesianImpedanceController: Exception getting state handle from interface: "
                            << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM(
                    "CartesianImpedanceController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM(
                        "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }



        dynamic_reconfigure_compliance_param_node_ =
                ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

        dynamic_server_compliance_param_ = std::make_unique<
                dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
                dynamic_reconfigure_compliance_param_node_);
        dynamic_server_compliance_param_->setCallback(
                boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));


        return true;
    }

    void CartesianImpedanceController::starting(const ros::Time& /*time*/) {
        // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
        // to initial configuration
        // Create the SetFullCollisionBehavior message
        franka::RobotState initial_state = state_handle_->getRobotState();
        // get jacobian
        std::array<double, 42> jacobian_array =
                model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        // convert to eigen
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        F_T_EE = initial_state.F_T_EE;
        EE_T_K = initial_state.EE_T_K;

        // set equilibrium point to current state
        position_d_ = initial_transform.translation();
        orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
        position_d_target_ = initial_transform.translation();
        orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

        // set nullspace equilibrium configuration to initial q
        //TODO: set controller paramter callback for dynamic reconfiguration
        q_d_nullspace_ = q_initial;
        nullspace_stiffness_target_ = 0.0001;
        K.topLeftCorner(3, 3) = 250 * Eigen::Matrix3d::Identity();
        K.bottomRightCorner(3, 3) << 80, 0, 0, 0, 80, 0, 0, 0, 10;
        D.topLeftCorner(3, 3) = 55 * Eigen::Matrix3d::Identity();
        D.bottomRightCorner(3, 3) << 18, 0, 0, 0, 18, 0, 0, 0, 6;
        cartesian_stiffness_target_ = K;
        cartesian_damping_target_ = D;
        max_I << 15.0, 15.0, 15.0, 8.0, 8.0, 1.0; // integrator saturation
        // uncomment the following lines if you wish to set a special inertia. Else (default) the inertia
        // is assumed to be equal to the robot inertia
        //T.topLeftCorner(3, 3) = 1 * Eigen::Matrix3d::Identity();
        //T.bottomRightCorner(3, 3) = 0.1 * Eigen::Matrix3d::Identity();

        //construct repulsing sphere around 0, 0, 0 as initializer. At first callback of hand position these values are set
        R = 0.2; C << 0.0, 0, 0.0;
        //free float
        //leave this commented out if you don't want to free float the end-effector
        /**
        K.setZero(); D.setZero(); repulsion_K.setZero(); repulsion_D.setZero(); nullspace_stiffness_target_ = 0;
        cartesian_stiffness_target_.setZero(); cartesian_damping_target_.setZero();
        **/


        //loggers
        std::ofstream F;
        F.open("/home/lucas/Desktop/MA/Force_Data/F_corrections.txt");
        F << "time Fref F_cmd Fx Fy Fz Mx My Mz F_imp_x F_imp_y F_imp_z F_imp_Mx F_imp_My F_imp_Mz\n";
        F.close();

        std::ofstream dtau;
        dtau.open("/home/lucas/Desktop/MA/Force_Data/dtau.txt");
        dtau << "time dtau1 dtau2 dtau3 dtau4 dtau5 dtau6 dtau7 dtau_abs\n";
        dtau.close();

        std::ofstream F_error;
        F_error.open("/home/lucas/Desktop/MA/Force_Data/friction.txt");
        F_error << "time eFx eFy eFz eMx eMy eMz f7\n";
        F_error.close();

        std::ofstream pose_error;
        pose_error.open("/home/lucas/Desktop/MA/Force_Data/pose_error.txt");
        pose_error << "time x y z rx ry rz xd yd zd rxd ryd rzd\n";
        pose_error.close();

        std::ofstream hand_tracking;
        hand_tracking.open("/home/lucas/Desktop/MA/Force_Data/hand_tracking.txt");
        hand_tracking << "time x_h y_h z_h rx ry rz Fx Fy Fz \n";
        hand_tracking.close();

        std::ofstream potential_field;
        potential_field.open("/home/lucas/Desktop/MA/Force_Data/potential_force.txt");
        potential_field << "time F_pot_x F_pot_y F_pot_z \n";
        potential_field.close();
    }


    void CartesianImpedanceController::update(const ros::Time& /*time*/,
                                                     const ros::Duration& /*period*/) {
        // get state variables
        franka::RobotState robot_state = state_handle_->getRobotState();
        std::array<double, 49> mass = model_handle_->getMass();
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        std::array<double, 42> jacobian_array =
                model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

        // convert to Eigen
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
                robot_state.tau_J_d.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.rotation());

        Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
        T = Lambda; // let robot behave with it's own physical inertia (How can we change the physical inertia and what does it mean?)

        // position error
        error.head(3) << position - position_d_;
        // orientation error
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.rotation() * error.tail(3);
        Eigen::Matrix<double, 6, 1> integrator_weights;
        integrator_weights << 150.0, 150.0, 150.0, 150.0, 150.0, 4.0; //give different DoF different integrator constants
        //only add movable degrees of freedom and only add when not free-floating and also do not add when in safety bubble
        I_error += (1-isInSphere) * Sm * dt* (1-control_mode) * integrator_weights.cwiseProduct(error);
        for (int i = 0; i < 6; i++){
            double a = I_error(i,0);
            I_error(i,0) = std::min(std::max(-max_I(i,0), a), max_I(i,0)); //saturation
        }

        // compute impedance control Force
        //F_impedance = -Lambda * T.inverse() * (D * (jacobian * dq) + K * error + integrator_weights.cwiseProduct(I_error));
        F_impedance = -1 * (D * (jacobian * dq) + K * error + I_error); //check for numerical issues, assume Lambda = T
        //Force PID
        F_ext = 0.9 * F_ext + 0.1 * Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data()); //low pass filter
        I_F_error += dt * Sf* (F_contact_des - F_ext); //+ in gazebo (-) on real robot //need to multiply with Sf here, else it gets accumulated nonstop
        F_cmd = 0.4 * (F_contact_des - F_ext) + 0.9 * I_F_error + 0.9 * F_contact_des; //F_contact_des is filtered from F_contact_target
        F_cmd = Sf * F_cmd;
        //F_cmd = F_contact_des;
        // allocate variables
        Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_impedance(7);
        // pseudoinverse for nullspace handling
        Eigen::MatrixXd jacobian_transpose_pinv;
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
        //construct external repulsion force
        r = position - C; // compute vector between EE and sphere
        double penetration_depth = std::max(0.0, R-r.norm());
        Eigen::Vector3d w = (jacobian*dq).head(3); //linear EE velocity
        rho = r.norm();
        rho_dot = 0.1 * (rho-rho_last)/dt + 0.9 * rho_dot; //if rho_dot > 0 we move the hand away from the end-effector and vice versa //add filtering
        rho_last = rho;
        r_dot = 0.001 * (r - r_last)/dt + 0.999 * r_dot; //filtering //we want to regulate r_dot to 0 if moving towards the hand
        r_last = r;
        //v = v.dot(r)/r.squaredNorm() * r; //projected velocity
        isInSphere = r.norm() < R;
        Eigen::Vector3d projected_error = error.head(3).dot(r)/r.squaredNorm() * r;
        double r_eq = 0.75 * R;
        /** update repulsive stiffnesses and damping **/
        //repulsion_K = (K.topLeftCorner(3,3) * r_eq/(R-r_eq))*Eigen::MatrixXd::Identity(3,3);
        //do not repulse in free float
        repulsion_K = K.topLeftCorner(3,3) * (error.head(3).cwiseAbs().asDiagonal())/(R-r_eq) + (1-control_mode) * 250 * Eigen::MatrixXd::Identity(3,3);
        //if rho_dot < 0 we go towards the hand and damp, if rho_dot > 0 we go away from the hand and do nothing
        if (rho_dot < 0.0){ repulsion_D = D.topLeftCorner(3,3); }
        else{ repulsion_D.setZero(); }
        if(isInSphere){
            I_error *= 0; //clear Integrator
            //choose damping as D(v_des - v_rel) with v_des = 0 and v_rel = r_dot
            //use case: Do not move when at target position : F_repulsion.head(3) = 0.1* (repulsion_K * penetration_depth * r/r.norm())
            // and repulsion_K = K.topLeftCorner(3,3) * (error.head(3).cwiseAbs().asDiagonal())/(R-r_eq); (scaled by error)
            F_repulsion.head(3) = 0.1* (repulsion_K * penetration_depth * r/r.norm()) + 0.9 * F_repulsion.head(3); //assume Theta = Lambda
        }
        else{ F_repulsion = 0.3 * 0.0 * F_repulsion + 0.7 * F_repulsion; } //command smooth slowdown



        // nullspace PD control with damping ratio = 1
        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                          jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q) - //if config_control = true we control the whole robot configuration
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);  // if config control ) false we don't care about the joint positions

        //virtual walls
        /**
        double wall_pos = 0.9;
        if (std::abs(position.y()) >= wall_pos){
            F_impedance.y() = -(500 * (position.y()-wall_pos)) + 45 *(jacobian*dq)(1,0) * 0.001 + 0.999 * F_impedance(1,0);
        }
        **/

        tau_impedance = jacobian.transpose() * Sm * (F_impedance + F_repulsion + F_potential) + jacobian.transpose() * Sf * F_cmd;
        tau_d << tau_impedance + tau_nullspace + coriolis; //add nullspace and coriolis components to desired torque
        tau_d << saturateTorqueRate(tau_d, tau_J_d);  // Saturate torque rate to avoid discontinuities
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d(i));
        }

        update_stiffness_and_references();

        //logging
        log_values_to_file(log_rate_() && do_logging);
        //log_values_to_file(log_rate_());
    }



}  // namespace force control

PLUGINLIB_EXPORT_CLASS(force_control::CartesianImpedanceController,
        controller_interface::ControllerBase)
