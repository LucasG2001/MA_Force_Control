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

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_example_controllers/compliance_paramConfig.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Vector3.h>


#define IDENTITY Eigen::MatrixXd::Identity(6,6)

namespace force_control {
    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServer;

    class CartesianImpedanceController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface> {
    public:
        CartesianImpedanceController() :
                action_server_node("cartesian_impedance_controller"),
                action_server_(action_server_node, "follow_joint_trajectory",boost::bind(&CartesianImpedanceController::action_callback, this, _1, &action_server_), false)
        {
            action_server_.start();
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

        Eigen::Matrix<double, 6,6> Lambda = IDENTITY; // operational space mass matrix
        Eigen::Matrix<double, 6, 6> Sm = IDENTITY; //task space selection matrix for positions and rotation
        Eigen::Matrix<double, 6, 6> Sf = Eigen::MatrixXd::Zero(6,6); //task space selection matrix for forces
        std::array<double, 16> F_T_EE; //end effector in flange frame
        std::array<double, 16> EE_T_K; //stiffness frame in EE frame
        Eigen::Affine3d pose_desired;
        Eigen::Matrix<double, 6, 1> error; //pose error (6d)
        Eigen::Matrix<double, 6, 1>  F_contact_des = Eigen::MatrixXd::Zero(6,6); //desired contact force
        Eigen::Matrix<double, 6, 1>  F_ext = Eigen::MatrixXd::Zero(6,6); //external forces
        Eigen::Matrix<double, 6, 1>  F_cmd = Eigen::MatrixXd::Zero(6,6); //commanded contact force
        Eigen::Matrix<double, 6, 1>  I_F_error = Eigen::MatrixXd::Zero(6,6); //force error integral
        Eigen::Matrix<double, 6,6> T = IDENTITY; // impedance inertia term
        Eigen::Matrix<double, 6,6> K = IDENTITY; //impedance stiffness term
        Eigen::Matrix<double, 6,6> D = IDENTITY; //impedance damping term
        Eigen::Matrix<double, 6,6> cartesian_stiffness_target_; //impedance damping term
        Eigen::Matrix<double, 6,6> cartesian_damping_target_; //impedance damping term
        Eigen::Matrix<double, 6,6> cartesian_inertia_target_; //impedance damping term

        //FLAGS
        bool config_control = true; //sets if we want to control the configuration of the robot in nullspace
        bool do_logging = false; //set if we do log values
        // end FLAGS
        double filter_params_{0.005};
        double nullspace_stiffness_{40};
        double nullspace_stiffness_target_{40.0};
        const double delta_tau_max_{1.0};
        Eigen::Matrix<double, 7, 1> q_d_nullspace_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        std::mutex position_and_orientation_d_target_mutex_;
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;
        double count = 0; //logging
        franka_hw::TriggerRate log_rate_{1000}; //logging
        double dt = 0.001;

        //repulsion sphere test;
        double R;
        Eigen::Vector3d C;
        Eigen::Matrix<double, 3, 3> repulsion_K, repulsion_D;
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

        // Equilibrium pose configuration
        ros::Subscriber sub_eq_config;
        void JointConfigCallback(const sensor_msgs::JointState& goal);

        // Hand Tracker
        ros::Subscriber sub_hand_pose;
        void HandPoseCallback(const geometry_msgs::Point& right_hand_pos);

        ros::NodeHandle action_server_node;
        ActionServer action_server_;
        void action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal, ActionServer* server);

        ros::Subscriber sub_force_action;
        void force_callback(const geometry_msgs::PoseConstPtr &goal_pose);

        ros::Subscriber sub_potential_field;
        void potential_field_callback(const geometry_msgs::Vector3 &goal_pose);


    };

}  // namespace franka_example_controllers

