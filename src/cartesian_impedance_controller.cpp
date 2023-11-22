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

        std::cout << "Do you want to enter test mode? (true/false)";
        std::cin >> test;
        std::cout << "Which joint do you want to test? (0-7)";
        std::cin >> joint;

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
        K.bottomRightCorner(3, 3) << 130, 0, 0, 0, 130, 0, 0, 0, 10;
        D.topLeftCorner(3, 3) = 60 * Eigen::Matrix3d::Identity();
        D.bottomRightCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 6;
        cartesian_stiffness_target_ = K;
        cartesian_damping_target_ = D;
        max_I << 30.0, 30.0, 30.0, 50.0, 50.0, 2.0; // integrator saturation
        // uncomment the following lines if you wish to set a special inertia. Else (default) the inertia
        // is assumed to be equal to the robot inertia
        //T.topLeftCorner(3, 3) = 1 * Eigen::Matrix3d::Identity();
        //T.bottomRightCorner(3, 3) = 0.1 * Eigen::Matrix3d::Identity();

        //construct repulsing sphere around 0, 0, 0 as initializer. At first callback of hand position these values are set
        R = 0.0000000001; C << 0.0, 0, 0.0;
        //free float
        //leave this commented out if you don't want to free float the end-effector
        /**
        K.setZero(); D.setZero(); repulsion_K.setZero(); repulsion_D.setZero(); nullspace_stiffness_target_ = 0;
        cartesian_stiffness_target_.setZero(); cartesian_damping_target_.setZero();
        **/

        //loggers
        // std::ofstream F;
        // F.open("/Home/Documents/BA/log/joint_0");
        // F << "time Fref F_cmd Fx Fy Fz Mx My Mz F_imp_x F_imp_y F_imp_z F_imp_Mx F_imp_My F_imp_Mz\n";
        // F.close();

        std::ofstream tau;
        tau.open("/home/viktor/Documents/BA/log/tau.txt");
        tau << "time tau1 tau2 tau3 tau4 tau5 tau6 tau7 tau_abs\n";
        tau.close();

        // std::ofstream F_error;
        // F_error.open("/Home/Documents/BA/log/joint_0");
        // F_error << "time eFx eFy eFz eMx eMy eMz f7\n";
        // F_error.close();

        // std::ofstream pose_error;
        // pose_error.open("/home/lucas/Desktop/MA/Force_Data/pose_error.txt");
        // pose_error << "time x y z rx ry rz xd yd zd rxd ryd rzd\n";
        // pose_error.close();
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
        integrator_weights << 75.0, 75.0, 75.0, 120.0, 120.0, 15.0; //give different DoF different integrator constants
        //only add movable degrees of freedom and only add when not free-floating and also do not add when in safety bubble
        I_error += (1-isInSphere) * integrator_weights.cwiseProduct(Sm * dt* error * (1-control_mode));
        for (int i = 0; i < 6; i++){
            double a = I_error(i,0);
            I_error(i,0) = std::min(std::max(-max_I(i,0), a), max_I(i,0)); //saturation
        }

        // compute impedance control Force
        //F_impedance = -Lambda * T.inverse() * (D * (jacobian * dq) + K * error + integrator_weights.cwiseProduct(I_error));
        F_impedance = -1 * (D * (jacobian * dq) + K * error + I_error); //check for numerical issues, assume Lambda = T
        //ROS_INFO_STREAM("CURRENT position is " << position.transpose());
        //ROS_INFO_STREAM("Integrator Force is " << I_error.transpose()); //Problem is here
        //ROS_INFO_STREAM("Impedance Force is " << F_impedance.transpose()); //Problem is here
        //Force PID
        F_ext = 0.9 * F_ext + 0.1 * Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data()); //low pass filter
        I_F_error += dt*(F_contact_des - F_ext); //+ in gazebo (-) on real robot
        F_cmd = 0.2 * (F_contact_des - F_ext) + 0.1 * I_F_error + F_contact_des; //no need to multiply with Sf since it is done afterwards anyway
        //F_cmd = F_contact_des;
        //ROS_INFO_STREAM("-------------------------------------------------------");
        // allocate variables
        Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_impedance(7);
        // pseudoinverse for nullspace handling
        Eigen::MatrixXd jacobian_transpose_pinv;
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
        //construct external repulsion force
        Eigen::Vector3d r = position - C; // compute vector between EE and sphere
        double penetration_depth = std::max(0.0, R-r.norm());
        Eigen::Vector3d v = (jacobian*dq).head(3);
        v = v.dot(r)/r.squaredNorm() * r; //projected velocity
        isInSphere = r.norm() < R;
        Eigen::Vector3d projected_error = error.head(3).dot(r)/r.squaredNorm() * r;
        double r_eq = 0.75 * R;
        //update repulsive stiffnesses
        repulsion_K = (K.topLeftCorner(3,3) * r_eq/(R-r_eq))*Eigen::MatrixXd::Identity(3,3); //30 for free floating operation, else use K
        repulsion_D = 3.0 * (repulsion_K).array().sqrt();
        // TODO smooth K changes

        if(isInSphere){
            F_repulsion.head(3) = 0.99* (repulsion_K * penetration_depth * r/r.norm() - repulsion_D * v) + 0.01 * F_repulsion.head(3); //assume Theta = Lambda
        }
        else{
            double decay = -log(0.0001)/R; //at 2R the damping is divided by 10'000
            F_repulsion.head(3) = - exp(decay * (R-r.norm())) * 0.1 * repulsion_D * v + 0.9 * F_repulsion.head(3); // 0.005 * F_repulsion_new + 0.995 * F_repulsion_old
        }


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

    
       Eigen::VectorXd tau_friction = calculate_tau_friction(dq); //Gets friction forces for current state


        tau_impedance = jacobian.transpose() * Sm * (F_impedance + F_repulsion + F_potential) + jacobian.transpose() * Sf * F_cmd;

        // If no movement wanted by impedance_force_control, no friction-dependend force will be applied
        for (int i = 0; i < 7; i++){
            if (std::abs(tau_impedance(i)) <= 0.1){
                tau_friction(i) = 0;
            }
        }

        //use for testing:

        if (test){ //Only set torques for the joint you want to test
            for (int i = 0; i < 7; i++){
                if (i != joint){
                    tau_d(i) = tau_nullspace(i) + coriolis(i);
                }
            }
        

            tau_d(joint) = timestamp * 0.2 + tau_nullspace(joint) + coriolis(joint);
        
            //If movement > 0, no torques applied anymore
            if (std::abs(dq(joint)) > .1){
            tau_d(joint) = tau_nullspace(joint) + coriolis(joint);
            timestamp = 0;
            }
        }
        else{
            tau_d << tau_impedance + tau_nullspace + coriolis/*  + tau_friction*/; //add nullspace, coriolis and friction components to desired torque
        }

        tau_d << saturateTorqueRate(tau_d, tau_J_d);  // Saturate torque rate to avoid discontinuities            

        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d(i));
        }

        update_stiffness_and_references();

        timestamp++;

        //logging
        //log_values_to_file(log_rate_() && do_logging);
        tau_measured = tau_J_d;
        vel_measured = dq;
        log_values_to_file(do_logging);
    }



}  // namespace force control

PLUGINLIB_EXPORT_CLASS(force_control::CartesianImpedanceController,
        controller_interface::ControllerBase)
