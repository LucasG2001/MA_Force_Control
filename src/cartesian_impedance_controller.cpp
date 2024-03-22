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
#define pGripperRight Eigen::Vector3d(0, -0.11, 0.07)
#define pGripperLeft Eigen::Vector3d(0, 0.11, 0.07)


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
                "reference_pose", 2, &CartesianImpedanceController::equilibriumPoseCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //control mode between free float and impedance
        sub_control_mode = node_handle.subscribe(
                "control_mode", 1, &CartesianImpedanceController::control_mode_callback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //subscriber for Yannic Hofmanns and Lucas Gimenos Hololens-Teleoperation code
        sub_eq_config = node_handle.subscribe(
                "joint_angles", 2, &CartesianImpedanceController::JointConfigCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //position of right hand for safety bubble
        sub_hand_pose = node_handle.subscribe(
                "right_hand", 2, &CartesianImpedanceController::HandPoseCallback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //movement with constrained motion in one direction, where force can be applied
        sub_force_action = node_handle.subscribe(
                "panda_force_action", 1, &CartesianImpedanceController::force_callback, this,
                ros::TransportHints().reliable().tcpNoDelay());
        //subscribes to the complete resulting force from planning scene potential field
        sub_potential_field = node_handle.subscribe(
                "/resulting_force", 2, &CartesianImpedanceController::potential_field_callback, this,
                ros::TransportHints().reliable().tcpNoDelay());
	    //subscribes to the complete resulting force from planning scene potential field
	    sub_impedance_param = node_handle.subscribe(
			    "impedance_param_reconfig", 2, &CartesianImpedanceController::impedance_param_reconfigure_callback, this,
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



        //dynamic_reconfigure_compliance_param_node_ =
          //      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

        //dynamic_server_compliance_param_ = std::make_unique<
          //      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
            //    dynamic_reconfigure_compliance_param_node_);
        //dynamic_server_compliance_param_->setCallback(
          //      boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));


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
		//q_d_nullspace_ = q_initial;
        //free float
        //leave this commented out if you don't want to free float the end-effector
        /**
        K.setZero(); D.setZero(); repulsion_K.setZero(); repulsion_D.setZero(); nullspace_stiffness_target_ = 0;
        cartesian_stiffness_target_.setZero(); cartesian_damping_target_.setZero();
        **/
        //loggers
		//ToDo: change this to another node
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
	    std::array<double, 16> joint7 = model_handle_-> getPose(franka::Frame::kJoint7);
        std::array<double, 49> mass = model_handle_->getMass();
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
	    std::array<double, 42> jac_j7 = model_handle_->getZeroJacobian(franka::Frame::kJoint7);
		//gripper control points
	    Eigen::Affine3d T_base_flange(Eigen::Matrix4d::Map(model_handle_-> getPose(franka::Frame::kFlange).data()));
	    Eigen::Vector3d gripper_left = T_base_flange.rotation() * pGripperLeft + T_base_flange.translation();
	    Eigen::Vector3d gripper_right = T_base_flange.rotation() * pGripperRight + T_base_flange.translation();

        // convert to Eigen
	    Eigen::Map<Eigen::Matrix<double, 3, 1>> joint7_position(joint7.data() + 12, 3);
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_j7(jac_j7.data());
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
		//T = 0.1 * Lambda; //lightweight robot
		//T = 5*Lambda; //heavy robot
        // position error
        error.head(3) << position - position_d_;
	    //Clamp the vector to a certain step size to not get infinite torques when goal is far away
	    double errorLowerBound = -0.1 * 250.0/K(0,0);
	    double errorUpperBound = 0.1 * 250.0/K(0,0);
	    // Apply clamping
	    error.head(3) = error.head(3).cwiseMax(errorLowerBound).cwiseMin(errorUpperBound);

	    // orientation error
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.rotation() * error.tail(3);

	    //Clamp the vector to a certain step size to not get infinite torques when goal is far away
	    double rotationLowerBound = -0.175 * 80/K(3,3);
	    double rotationUpperBound = 0.175 * 80/K(3,3);
	    // Apply clamping
	    error.tail(3) = error.tail(3).cwiseMax(rotationLowerBound).cwiseMin(rotationUpperBound);


        //only add movable degrees of freedom and only add when not free-floating and also do not add when in safety bubble
        I_error +=  dt * Sm * (1-control_mode) * integrator_weights.cwiseProduct(error);
        for (int i = 0; i < 6; i++){
            double a = I_error(i,0);
            I_error(i,0) = std::min(std::max(-max_I(i,0), a), max_I(i,0)); //saturation
        }
        //Force PID
        F_ext = 0.9 * F_ext + 0.1 * Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data()); //low pass filter
        I_F_error += dt * (F_contact_des - F_ext); //+ in gazebo (-) on real robot //need to multiply with Sf here, else it gets accumulated nonstop
        F_cmd = 0.4 * (F_contact_des - F_ext) + 0.9 * I_F_error + 0.9 * F_contact_des; //F_contact_des is filtered from F_contact_target
	    // compute impedance control Force (simplified control law Lambda = Theta)

        // allocate variables
        Eigen::VectorXd tau_nullspace(7), tau_d(7), tau_impedance(7);
        // pseudoinverse for nullspace handling
        Eigen::MatrixXd jacobian_transpose_pinv, N, N_pinv, positional_jacobian, Jp_T_pinv;
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
        //construct external repulsion force
		//we check four points
		Eigen::Vector3d control_points[4];
		int min_index = 0;
		Eigen::Matrix<double, 4, 1> distances;
	    control_points[0] = position - C; //End effector
		control_points[1]  = joint7_position - C; //Franka coordinate system joint 7
	    control_points[2]  = gripper_left - C; //left gripper point (seen from robot)
	    control_points[3]  = gripper_right - C; //left gripper point (seen from robot)
		r = control_points[0];
		for (int i = 1; i < 4; i++){
			if(control_points[i].norm() < r.norm()){ r = control_points[i]; min_index = i;} //get smallest r
		}
	    //get projected radial velocity
	    Eigen::Vector3d v = ((jacobian * dq).head(3)).dot(r) * 1/r.squaredNorm() * r;
	    double penetration_depth = std::max(0.0, R-r.norm());
		//calculate repulsive torques
	    if(r.norm() <= R){
		    //I_error =  0.999 * I_error; //EMA filter for I_error
		    F_repulsion.head(3) = repulsion_K * penetration_depth * r/(r.norm()) - repulsion_D * v;
		    if (min_index == 1){
			    //use joint 7 jacobian if joint 7 is detected as nearest
			    tau_repulsion = 0.1 * (jacobian_j7.transpose() * F_repulsion) + 0.9 * tau_repulsion; // EMA for repulsive tau
		    }
		    else {
			    tau_repulsion = 0.1 * (jacobian.transpose() * F_repulsion) + 0.9 * tau_repulsion; //EMA for repulsive tau
		    }

	    }
	    else{ tau_repulsion = 0.3 * 0 * tau_repulsion + 0.7 * tau_repulsion; } //command smooth slowdown


	    //ToDo: Why is I_error negative (same as error in F_impedance)?
	    F_impedance = -1 * (D * (jacobian * dq - velocity_d_) + K * error + I_error);
		//full impedance control law
	    //F_impedance = (Lambda*T.inverse() - IDENTITY) * -F_ext - Lambda*T.inverse()*(D * (jacobian * dq) + K * error + I_error);

		//Singularity avoidance
		double V, m, m0, k;
		m0 = 0.0; k = 2.5;
		m = std::sqrt((jacobian*jacobian.transpose()).determinant());
		V = k * (m - m0) * (m - m0);
		//std::cout << "Potential is " << V << " and q is " << q.transpose() << "\n";
		/**
		//Nullspace projection did not work as expected
	    Eigen::Matrix<double, 6, 1> F_imp_pos, F_imp_or;
	    //F_imp_pos.head(3) = F_impedance.head(3); F_imp_pos.tail(3) << 0, 0, 0;
	    //F_imp_or.tail(3) = F_impedance.tail(3); F_imp_or.head(3) << 0, 0, 0;


        // nullspace PD control with damping ratio = 1
	    // Extract the top half (3x7 matrix)
	    positional_jacobian = jacobian.block<3, 7>(0, 0); //extract positional jacobian
	    pseudoInverse(positional_jacobian.transpose(), Jp_T_pinv);

	    N = Eigen::MatrixXd::Identity(7, 7) - positional_jacobian.transpose() * Jp_T_pinv;
	    pseudoInverse(N, N_pinv);
		**/

		N = Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv;
		tau_nullspace << N *(nullspace_stiffness_ * (q_d_nullspace_ - q) - //control to neutral configuration
						  (2.0 * sqrt(nullspace_stiffness_)) * dq);  // if config control ) false we don't care about the joint positions
	    tau_nullspace = tau_nullspace.cwiseProduct(nullspace_weights); //weigh different joint differently

		//tau_nullspace << N * -(A * q.cwiseProduct(q) + B * q + c);

        tau_impedance = jacobian.transpose() * Sm * (F_impedance + F_potential) + jacobian.transpose() * Sf * F_cmd; //first priority task
        tau_d << tau_impedance + tau_nullspace + tau_repulsion + coriolis; //add nullspace and coriolis components to desired torque
        tau_d << saturateTorqueRate(tau_d, tau_J_d);  // Saturate torque rate to avoid discontinuities
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d(i));
        }

		//logging and update
        update_stiffness_and_references();
        log_values_to_file(log_rate_() && do_logging);
    }



}  // namespace force control

PLUGINLIB_EXPORT_CLASS(force_control::CartesianImpedanceController,
        controller_interface::ControllerBase)
