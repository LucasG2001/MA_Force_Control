//
// Created by lucas on 16.06.23.
//

#ifndef FORCE_CONTROL_CARTESIAN_IMPEDANCE_CONTROLLER_H
#define FORCE_CONTROL_CARTESIAN_IMPEDANCE_CONTROLLER_H

#endif //FORCE_CONTROL_CARTESIAN_IMPEDANCE_CONTROLLER_H
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <franka_hw/trigger_rate.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_example_controllers/compliance_paramConfig.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <../../../devel/include/goal_state_publisher/testMsg.h>


#define IDENTITY Eigen::MatrixXd::Identity(6,6)

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
            moveit_action_server.start();
        }

        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void update_stiffness_and_references();
        void log_values_to_file(bool do_logging);
        void load_friction_parameters(const std::string& filePath);
        void calculate_tau_friction();
        void state_observer();
        void state_tuner();

    private:
        // Saturation
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(
                const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;

        Eigen::Matrix<double, 6,6> Lambda = IDENTITY; // operational space mass matrix
        Eigen::Matrix<double, 6, 6> Sm = IDENTITY; //task space selection matrix for positions and rotation
        Eigen::Matrix<double, 6, 6> Sf = Eigen::MatrixXd::Zero(6,6); //task space selection matrix for forces
        std::array<double, 16> F_T_EE; //end effector in flange frame
        std::array<double, 16> EE_T_K; //stiffness frame in EE frame
        Eigen::Affine3d pose_desired;
        Eigen::Matrix<double, 6, 1> error; //pose error (6d)
        Eigen::Matrix<double, 6, 1> I_error = Eigen::MatrixXd::Zero(6,1); //pose error (6d)
        Eigen::Matrix<double, 6, 1> max_I = Eigen::MatrixXd::Zero(6,1); //pose error (6d)
        Eigen::Matrix<double, 7, 1> tau_error = Eigen::MatrixXd::Zero(7,1);//
        Eigen::Matrix<double, 6, 1>  F_contact_des = Eigen::MatrixXd::Zero(6,1); //desired contact force
        Eigen::Matrix<double, 6, 1>  F_ext = Eigen::MatrixXd::Zero(6,1); //external forces
        Eigen::Matrix<double, 6, 1>  F_cmd = Eigen::MatrixXd::Zero(6,1); //commanded contact force
        Eigen::Matrix<double, 6, 1>  I_F_error = Eigen::MatrixXd::Zero(6,1); //force error integral
        Eigen::Matrix<double, 6,6> T = IDENTITY; // impedance inertia term
        Eigen::Matrix<double, 6,6> K = IDENTITY; //impedance stiffness term
        Eigen::Matrix<double, 6,6> D = IDENTITY; //impedance damping term
        Eigen::Matrix<double, 6,6> cartesian_stiffness_target_; //impedance damping term
        Eigen::Matrix<double, 6,6> cartesian_damping_target_; //impedance damping term
        Eigen::Matrix<double, 6,6> cartesian_inertia_target_; //impedance damping term
        Eigen::Matrix<double, 6,7> jacobian; //jacobian matrix of robot
        Eigen::MatrixXd jacobian_transpose_pinv;
        Eigen::MatrixXd jacobian_pinv;
        Eigen::MatrixXd N;
        Eigen::Matrix<double, 7,1> coriolis; //coriolis torques of robot
        Eigen::Matrix<double, 7, 7> M; //Mass-matrix
        Eigen::Matrix<double, 7,1> pos_measured = Eigen::MatrixXd::Zero(7,1); //Measured position for logging
        Eigen::Matrix<double, 7,1> tau_d = Eigen::MatrixXd::Zero(7,1); //commanded torque
        Eigen::Matrix<double, 7, 1> tau_J_d = Eigen::MatrixXd::Zero(7,1); //measured torque
        Eigen::Matrix<double, 7, 1> tau_nullspace = Eigen::MatrixXd::Zero(7,1); //nullspace torque
        Eigen::Matrix<double, 7, 1> dq = Eigen::MatrixXd::Zero(7,1); //measured rotational speed
        Eigen::Matrix<double, 7, 1> dq_filtered = Eigen::MatrixXd::Zero(7,1); //rotational speed filtered for friction compensation
        Eigen::Matrix<double, 7, 1> dq_d = Eigen::MatrixXd::Zero(7,1); //desired rotational speed
        Eigen::Matrix<double, 7, 1> dq_imp = Eigen::MatrixXd::Zero(7,1); //"impedance dq", dq without the nullspace-part of it
        Eigen::Matrix<double, 7, 1> q = Eigen::MatrixXd::Zero(7,1); //position of joint measured
        Eigen::Matrix<double, 7, 1> gravity = Eigen::MatrixXd::Zero(7,1); //gravity vector
        Eigen::Matrix<double, 7, 1> tau_impedance = Eigen::MatrixXd::Zero(7,1); //torque for every joint from Jacobi * F_cmd
        Eigen::Matrix<double, 7, 1> tau_impedance_filtered = Eigen::MatrixXd::Zero(7,1); //filtered impedance torque for friction compensation
        Eigen::Matrix<double, 7, 1> tau_friction = Eigen::MatrixXd::Zero(7,1); //torque compensating friction
        const Eigen::VectorXd sigmoid_param = (Eigen::VectorXd(7) << -2400, -200, -800, -1200, -1600, -1600, -200).finished(); 
        Eigen::Matrix<double, 6, 1> F_friction_keep = Eigen::MatrixXd::Zero(6,1);

        const Eigen::VectorXd error_goal =  (Eigen::VectorXd(6) << .001, .001, .001, .001, .001, .01).finished(); //Sufficient good errors needed for friction compensation
        Eigen::Matrix<double, 6, 1> error_threshold; 
        Eigen::Matrix<double, 7, 1> tau_threshold = Eigen::MatrixXd::Zero(7,1); //Minimum tau_impedance, after which friction compensation should turn on
        const Eigen::DiagonalMatrix<double, 6> error_goal_separate = error_goal.asDiagonal(); //Diagonal matrix with every error_goal in a separate column
        Eigen::Matrix<double, 7, 6> tau_threshold_separate = Eigen::MatrixXd::Zero(7,6); //separated tau_thresholds (every error with own column)
        Eigen::Matrix<double, 7, 1> tau_threshold_min = Eigen::MatrixXd::Zero(7,1); //values used for comparison form tau_threshold_separate
        Eigen::Matrix<bool, 6, 1> error_goal_met; //compares for every degree of freedom whether error goal is met

        Eigen::Matrix<double, 7, 1> coulomb_friction = Eigen::MatrixXd::Zero(7,1); //coulomb friction parameters imported from lists/friction_parameters.txt
        Eigen::Matrix<double, 7, 1> offset_friction = Eigen::MatrixXd::Zero(7,1); //offset of friction in one direction
        Eigen::Matrix<double, 7, 1> static_friction_minus = Eigen::MatrixXd::Zero(7,1); //static friction in negative direction
        Eigen::Matrix<double, 7, 1> lin_a;//component a of linear friction model (a + b*dq)
        Eigen::Matrix<double, 7, 1> lin_b;//component b of linear friction model (a + b*dq)
        Eigen::Matrix<double, 7, 1> qua_a;//component a of quadratic friction model (a + b*dq + c*dq²)
        Eigen::Matrix<double, 7, 1> qua_b;//component b of quadratic friction model (a + b*dq + c*dq²)
        Eigen::Matrix<double, 7, 1> qua_c;//component c of quadratic friction model (a + b*dq + c*dq²)
        Eigen::MatrixXi friction_state = Eigen::MatrixXi::Zero(7,1); //current friction state (0 == off, 1 == static, 2 == quadratic, 3 == linear)

        //state observer stuff

        Eigen::Matrix<double, 7, 1> integral_observer = Eigen::MatrixXd::Zero(7,1);
        Eigen::Matrix<double, 7, 1> r = Eigen::MatrixXd::Zero(7,1);
        Eigen::Matrix<double, 7, 7> M_old = Eigen::MatrixXd::Zero(7,7);
        const Eigen::Matrix<double, 7, 7> K_0 = (Eigen::VectorXd(7) << 10, 10, 10, 10, 10, 10, 10).finished().asDiagonal();
        Eigen::Matrix<double, 7, 1> tau_external = Eigen::MatrixXd::Zero(7,1);

        //state tuner stuff

        Eigen::Matrix<double, 7, 1> dz = Eigen::MatrixXd::Zero(7,1);
        Eigen::Matrix<double, 7, 1> z = Eigen::MatrixXd::Zero(7,1);
        Eigen::Matrix<double, 7, 1> g = Eigen::MatrixXd::Zero(7,1);
        Eigen::Matrix<double, 7, 1> f = Eigen::MatrixXd::Zero(7,1);
        const Eigen::Matrix<double, 7, 1> sigma_0 = (Eigen::VectorXd(7) << 80.8, 59.18, 42.16, 83.64, 32.8, 21.83, 0.53).finished();
        const Eigen::Matrix<double, 7, 1> sigma_1 = (Eigen::VectorXd(7) << 0.0012728, 0.001, 0.001481, 0.00138, 0.0016, 0.000755, 0.000678).finished();
        Eigen::Matrix<double, 7, 1> friction_optimized = Eigen::MatrixXd::Zero(7,1);
        double x_start = 0;
        double x_integral = 0;
        double z_guess = 0;
        double sigma_0_guess = 0;
        Eigen::Matrix<double, 7, 1> dq_old = Eigen::MatrixXd::Zero(7,1);
        Eigen::Matrix<double, 7, 1> sigma_1_guess = Eigen::MatrixXd::Zero(7,1);


        //FLAGS
        bool config_control = false; //sets if we want to control the configuration of the robot in nullspace
        bool do_logging = true; //set if we do log values
        bool test = false; //Set if you want to test particular joint
        bool friction = false; //Sets whether friction compensation is enabled
        int joint = 0; //Number of joint to test
        int timestamp = 0; //helper for linear increase of test torque
        // end FLAGS
        double filter_params_{0.005};
        double nullspace_stiffness_{0.1};
        double nullspace_stiffness_target_{0.1};
        const double delta_tau_max_{1.0}; //max. torque-rate to ensure continuity
        Eigen::Matrix<double, 7, 1> q_d_nullspace_; //neutral pose;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        std::mutex position_and_orientation_d_target_mutex_;
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;
        unsigned int count = 0; //logging
        franka_hw::TriggerRate log_rate_{50}; //logging
        const double dt = 0.001;

        //repulsion sphere around right hand;
        bool isInSphere = false;
        double R;
        Eigen::Vector3d C;
        Eigen::Matrix<double, 3, 3> repulsion_K, repulsion_D;
        //all included Forces
        Eigen::Matrix<double, 6, 1> F_repulsion;
        Eigen::Matrix<double, 6, 1> F_potential = Eigen::MatrixXd::Zero(6,1);
        Eigen::Matrix<double, 6, 1> F_impedance;

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
        void HandPoseCallback(const geometry_msgs::Point& right_hand_pos);

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
        //subscriber for test mode
        ros::Subscriber sub_test;
        void test_callback(const goal_state_publisher::testMsg::ConstPtr& msg);
        //subscriber for friction mode
        ros::Subscriber sub_friction;
        void friction_callback(const std_msgs::Bool::ConstPtr& msg);
        

    };

}  // namespace franka_example_controllers

