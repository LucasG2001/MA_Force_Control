//
// Created by lucas on 16.06.23.
//

#ifndef FORCE_CONTROL_CARTESIAN_IMPEDANCE_CONTROLLER_H
#define FORCE_CONTROL_CARTESIAN_IMPEDANCE_CONTROLLER_H

#endif
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <fstream>
#include <ostream>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <franka_hw/trigger_rate.h>
#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int16.h>
#include <custom_msgs/ImpedanceParameterMsg.h>

#define IDENTITY Eigen::MatrixXd::Identity(6,6)
#define PI 3.1415926536
namespace force_control {
    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServer;

    class CartesianImpedanceController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface> {
    public:
        CartesianImpedanceController() :
                moveit_action_server_node("cartesian_impedance_controller"),
                moveit_action_server(moveit_action_server_node, "follow_joint_trajectory", boost::bind(&CartesianImpedanceController::action_callback, this, _1, &moveit_action_server), false)
        {
	        integrator_weights << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //give different DoF different integrator constants
			max_I << 1.0, 1.0, 1.0, 1.0, 1.0, 0.5; //  saturation
	        nullspace_stiffness_target_ = 0;    
	        K.topLeftCorner(3, 3) = 300.0 * Eigen::Matrix3d::Identity();
	        K.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 10;
	        D.topLeftCorner(3, 3) = 40 * Eigen::Matrix3d::Identity();
	        D.bottomRightCorner(3, 3) << 18, 0, 0, 0, 18, 0, 0, 0, 7;
	        repulsion_K = K.topLeftCorner(3,3) * 0.8;
	        repulsion_D = D.topLeftCorner(3,3) * 0.3;
	        cartesian_stiffness_target_ = K;
	        cartesian_damping_target_ = D;
			repulsion_K_target_ = repulsion_K;
			repulsion_D_target_ = repulsion_D;
			nullspace_weights << 0.95, 0.95, 0.95, 0.95, 0.13, 0.13, 0.02;
			q_d_nullspace_ << 0.0, -PI/4, 0, -3/4*PI, 0.0, PI/2, PI/4; //neutral pose
			//construct repulsing sphere around 0, 0, 0 as initializer. At first callback of hand position these values are set
	        R = 0.01; C << 0.0, 0, 0.0;
			moveit_action_server.start();
			//Nullspace handling
			Eigen::Matrix<double, 7, 1> a_diagonal, b_diagonal;
			a_diagonal << 3.57383,9.6541750,3.57383355,13.31557,0.4765,1.125,0.0595;
			b_diagonal << 0, 0, 0, 41.8322051343963, 0, -4.20463100422855, 0;
			c << 0, 0, 0, 32.85501, 0, 3.9261, 0;
			A = a_diagonal.asDiagonal()*0.5; B = b_diagonal.asDiagonal()*0.5; c = c * 0.5;

        }

        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void update_stiffness_and_references();
        void log_values_to_file(bool do_logging);

    private:
        // Saturation
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(
                const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;

		//Physical quantities
        Eigen::Matrix<double, 6,6> Lambda = IDENTITY; // operational space mass matrix
        Eigen::Matrix<double, 6, 6> Sm = IDENTITY; //task space selection matrix for positions and rotation
	    Eigen::Matrix<double, 6, 7> J; //EE geometric Jacobian
        Eigen::Matrix<double, 6, 6> Sf = Eigen::MatrixXd::Zero(6,6); //task space selection matrix for forces
        std::array<double, 16> F_T_EE; //end effector in flange frame
        std::array<double, 16> EE_T_K; //stiffness frame in EE frame
        Eigen::Affine3d pose_desired;
        Eigen::Matrix<double, 6, 1> error; //pose error (6d)
        Eigen::Matrix<double, 6, 1> I_error = Eigen::MatrixXd::Zero(7,1); //pose error (6d)
        Eigen::Matrix<double, 6, 1> max_I; //maximum Integrator windup
	    Eigen::Matrix<double, 6, 1> integrator_weights;
        Eigen::Matrix<double, 6, 1>  F_contact_des = Eigen::MatrixXd::Zero(6,1); //desired contact force
	    Eigen::Matrix<double, 6, 1>  F_contact_target = Eigen::MatrixXd::Zero(6,1); //desired contact force used for filtering
        Eigen::Matrix<double, 6, 1>  F_ext = Eigen::MatrixXd::Zero(6,1); //external forces
        Eigen::Matrix<double, 6, 1>  F_cmd = Eigen::MatrixXd::Zero(6,1); //commanded contact force
        Eigen::Matrix<double, 6, 1>  I_F_error = Eigen::MatrixXd::Zero(6,1); //force error integral

		//all IMPEDANCE PARAMETERS
		//Robot
        Eigen::Matrix<double, 6,6> T = IDENTITY; // impedance inertia term
        Eigen::Matrix<double, 6,6> K = IDENTITY; //impedance stiffness term
        Eigen::Matrix<double, 6,6> D = IDENTITY; //impedance damping term
        Eigen::Matrix<double, 6,6> cartesian_stiffness_target_; //impedance damping term
        Eigen::Matrix<double, 6,6> cartesian_damping_target_; //impedance damping term
        Eigen::Matrix<double, 6,6> cartesian_inertia_target_; //impedance damping term
		// Safety bubble
		Eigen::Matrix<double, 3, 3> repulsion_K, repulsion_D;
	    Eigen::Matrix<double, 3,3> repulsion_K_target_, repulsion_D_target_; //impedance damping term
        //FLAGS
        bool config_control = false; //sets if we want to control the configuration of the robot in nullspace
        bool do_logging = true; //set if we do log values
        // end FLAGS
        double filter_params_{0.005};
		//singularity handling
        double nullspace_stiffness_{0};
        double nullspace_stiffness_target_{0};
		Eigen::Matrix<double, 7, 1> nullspace_weights;
		//we compute tau nullspace as a quadratic cost from the joint limits, i.e. tau_null = q^TAq + Bq + C
		Eigen::Matrix<double, 7, 7> A, B;
		Eigen::Matrix<double, 7, 1> c;
	    Eigen::Matrix<double, 7, 1> dq_;
        const double delta_tau_max_{1.0};
        Eigen::Matrix<double, 7, 1> q_d_nullspace_;
        Eigen::Vector3d position_d_;
	    Eigen::Matrix<double, 6, 1> velocity_d_;
	    Eigen::Matrix<double, 6, 1> velocity_d_target_;
        Eigen::Quaterniond orientation_d_;
        std::mutex position_and_orientation_d_target_mutex_;
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;
        double count = 0; //logging
        franka_hw::TriggerRate log_rate_{250}; //logging
        double dt = 0.001;

        //repulsion sphere around right hand;
        bool isInSphere = false;
        double R; //safety bubble radius
        Eigen::Vector3d r; //distance of nearest control point to safety bubble position - C
        Eigen::Vector3d C;
        //all included Forces
        Eigen::Matrix<double, 6, 1> F_repulsion, F_potential, F_impedance;
		Eigen::Matrix<double, 7, 1> tau_repulsion;

        // Dynamic reconfigure
        std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
                dynamic_server_compliance_param_;
        ros::NodeHandle dynamic_reconfigure_compliance_param_node_;

        void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                                     uint32_t level);

        // Equilibrium pose subscriber
        ros::Subscriber sub_equilibrium_pose_;
        void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

        // Control mode subscriber
        int control_mode = 0; // 0 for normal 1 for free float
        ros::Subscriber sub_control_mode;
        void control_mode_callback(const std_msgs::Int16ConstPtr & msg);

        // Equilibrium pose configuration
        ros::Subscriber sub_eq_config;
        void JointConfigCallback(const sensor_msgs::JointState& goal);

        // Hand Tracker
        ros::Subscriber sub_hand_pose;
        void HandPoseCallback(const geometry_msgs::Pose& right_hand_pose);

        //moveit action server to execute path planning
        ros::NodeHandle moveit_action_server_node;
        ActionServer moveit_action_server;
        void action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal, ActionServer* server);

        //subscriber for forcing actions (Force + movement)
        ros::Subscriber sub_force_action;
        void force_callback(const geometry_msgs::PoseStampedConstPtr &goal_pose);

        //subscriber for scene potential field
        ros::Subscriber sub_potential_field;
        void potential_field_callback(const geometry_msgs::Vector3 &goal_pose);

		//subscriber for impedance parameter updating
	    ros::Subscriber sub_impedance_param;
	    void impedance_param_reconfigure_callback(const custom_msgs::ImpedanceParameterMsgConstPtr &msg);

    };

}  // namespace franka_example_controllers

