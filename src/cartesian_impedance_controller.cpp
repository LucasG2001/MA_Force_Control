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

        std::vector<double> cartesian_stiffness_vector;
        std::vector<double> cartesian_damping_vector;

        sub_equilibrium_pose_ = node_handle.subscribe(
                "reference_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());

        sub_control_mode = node_handle.subscribe(
                "control_mode", 20, &CartesianImpedanceController::control_mode_callback, this,
                ros::TransportHints().reliable().tcpNoDelay());

        //subscriber for Yannic Hofmanns and Lucas Gimenos Hololens-Teleoperation code
        sub_eq_config = node_handle.subscribe(
                "joint_angles", 20, &CartesianImpedanceController::JointConfigCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());

        sub_hand_pose = node_handle.subscribe(
                "right_hand", 20, &CartesianImpedanceController::HandPoseCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());

        sub_force_action = node_handle.subscribe(
                "panda_force_action", 20, &CartesianImpedanceController::force_callback, this,
                ros::TransportHints().reliable().tcpNoDelay());

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


        position_d_.setZero();
        orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        position_d_target_.setZero();
        orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

        return true;
    }

    void CartesianImpedanceController::starting(const ros::Time& /*time*/) {
        // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
        // to initial configuration
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
        q_d_nullspace_ = q_initial;
        nullspace_stiffness_target_ = 30;
        K.topLeftCorner(3, 3) = 200 * Eigen::Matrix3d::Identity();
        K.bottomRightCorner(3, 3) << 90, 0, 0, 0, 90, 0, 0, 0, 80;
        D.topLeftCorner(3, 3) = 35 * Eigen::Matrix3d::Identity();
        D.bottomRightCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 12;
        cartesian_stiffness_target_ = K;
        cartesian_damping_target_ = D;
        //T.topLeftCorner(3, 3) = 1 * Eigen::Matrix3d::Identity();
        //T.bottomRightCorner(3, 3) = 0.1 * Eigen::Matrix3d::Identity();

        //construct repulsing sphere around 0, 0, 0
        R = 0.00001; C << 0.0, 0, 0.0;
        repulsion_K.setZero(); repulsion_D.setZero();
        //set zero stiffness and damping for rotational velocities and positions
        repulsion_K = Eigen::Matrix3d::Identity(); repulsion_D = Eigen::Matrix3d::Identity();



        //free float
        /**
        K.setZero(); D.setZero(); repulsion_K.setZero(); repulsion_D.setZero(); nullspace_stiffness_target_ = 0;
        cartesian_stiffness_target_.setZero(); cartesian_damping_target_.setZero();
        **/

        //loggers
        std::ofstream F;
        F.open("/home/lucas/Desktop/MA/Force_Data/F_corrections.txt");
        //F << "time,Fx,Fy,Fz,Mx,My,Mz,w_dot_des\n";
        F << "time dF1 dF2 dF3 dF4 dF5 dF6 Fref Fcmd kalman_estimate Fx Fy Fz Mx My Mz\n";
        F.close();

        std::ofstream dtau;
        dtau.open("/home/lucas/Desktop/MA/Force_Data/dtau.txt");
        dtau << "time dtau1 dtau2 dtau3 dtau4 dtau5 dtau6 dtau7 dtau_abs\n";
        dtau.close();

        std::ofstream F_error;
        F_error.open("/home/lucas/Desktop/MA/Force_Data/friction.txt");
        F_error << "time eFx eFy eFz eMx eMy eMz f7\n";
        F_error.close();
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
        T = Lambda; // let robot behave with it's own physical inertia
        // compute error to desired pose
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

        // compute control
        F_impedance = -Lambda * T.inverse() * (D * (jacobian * dq) + K * error);

        //Force PID
        F_ext = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data()) * 0.999 + 0.001 * F_ext;
        I_F_error += dt*(F_contact_des - F_ext);
        F_cmd = 0.2 * (F_contact_des - F_ext) + 0.1 * I_F_error + F_contact_des - 0 *Sf * F_impedance; //no need to multiply with Sf since it is done afterwards anyway
        //F_cmd = F_contact_des;

        // allocate variables
        Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_impedance(7);

        // pseudoinverse for nullspace handling
        // kinematic pseuoinverse
        Eigen::MatrixXd jacobian_transpose_pinv;
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

        //construct external repulsion force
        Eigen::Vector3d r = position - C; // compute vector between EE and sphere
        //ROS_INFO_STREAM("r is " << r.transpose());
        double penetration_depth = std::max(0.0, R-r.norm());
        //ROS_INFO_STREAM("penetration depth is " << penetration_depth);
        Eigen::Vector3d v = (jacobian*dq).head(3);
        v = v.dot(r)/r.squaredNorm() * r; //projected velocity
        //ROS_INFO_STREAM("projected velocity is " << v.transpose());
        bool isInSphere = r.norm() < R;
        //double alpha = error.norm()/(r.norm()+0.0001); //scaling to reach same forces in repulsion and add offset to not divide by 0
        Eigen::Vector3d projected_error = error.head(3).dot(r)/r.squaredNorm() * r;
        double r_eq = 0.8 * R;
        //double alpha = projected_error.norm()*R*0.95;
        //repulsion_K = (K * r_eq/(R-r_eq)).topLeftCorner(3,3); //assume Lambda = Theta(T) to avoid numerical issues
        repulsion_K = (K.topLeftCorner(3,3) * r_eq/(R-r_eq))*Eigen::MatrixXd::Identity(3,3); //30 for free floating operation, else use K
        repulsion_D = 2 * (repulsion_K).array().sqrt();

        if(isInSphere){
            F_repulsion.head(3) = 0.99* (repulsion_K * penetration_depth * r/r.norm() - repulsion_D * v) + 0.01 * F_repulsion.head(3); //assume Theta = Lambda
        }
        else{
            double decay = -log(0.0001)/R; //at 2R the damping is divided by 10'000
            F_repulsion.head(3) = - exp(decay * (R-r.norm())) * 0.1 * repulsion_D * v + 0.9 * F_repulsion.head(3); // 0.005 * F_repulsion_new + 0.995 * F_repulsion_old
        }

        /**
        ROS_INFO_STREAM("impedance Force is " << F_impedance.transpose());
        ROS_INFO_STREAM("repulsive Force is " << F_repulsion.transpose());
        ROS_INFO_STREAM("Is in sphere is " << isInSphere);
        ROS_INFO_STREAM("R is " << r.norm());
        **/

        // nullspace PD control with damping ratio = 1
        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                          jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q) - //Do not use joint positions yet
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);

        //virtual walls
        double wall_pos = 12.0;
        if (std::abs(position.y()) >= wall_pos){
            F_impedance.y() = -(500 * (position.y()-wall_pos)) + 45 *(jacobian*dq)(1,0) * 0.001 + 0.999 * F_impedance(1,0);
        }

        tau_impedance = jacobian.transpose() * Sm * (F_impedance + F_repulsion + F_potential) + jacobian.transpose() * Sf * F_cmd;
        tau_d << tau_impedance + tau_nullspace + coriolis; //add nullspace and coriolis components to desired torque
        tau_d << saturateTorqueRate(tau_d, tau_J_d);  // Saturate torque rate to avoid discontinuities
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d(i));
        }

        update_stiffness_and_references();

        //logging
        log_values_to_file(log_rate_() && do_logging);
    }



}  // namespace force control

PLUGINLIB_EXPORT_CLASS(force_control::CartesianImpedanceController,
        controller_interface::ControllerBase)
