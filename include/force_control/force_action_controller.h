//
// Created by lucas on 17.04.23.

//
// Created by lucas on 16.04.23.
//
#ifndef FORCE_ACTION_CONTROLLER_H
#define FORCE_ACTION_CONTROLLER_H

#include "ros/ros.h"
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <chrono>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iostream>
#include <fstream>



/* ToDo: Implement w_dot_desired control */
namespace force_control {

    struct Reference{
        Eigen::Matrix<double, 6, 1>  F;
        Eigen::Matrix<double, 6, 1>  w;
        Eigen::Affine3d pose;
        Eigen::Matrix<double, 3, 1>  position;
        Eigen::Matrix<double, 3, 1>  orientation;
        Eigen::Matrix<double, 7, 1>  q;
        Eigen::Matrix<double, 7, 1>  dq;
    };

    struct State{
        Eigen::Matrix<double, 6, 1> F;
        Eigen::Matrix<double, 6, 1> w;
        Eigen::Matrix<double, 4, 4> pose;
        Eigen::Matrix<double, 6, 1> r;
        Eigen::Matrix<double, 7, 1> q;
        Eigen::Matrix<double, 7, 1> dq;
        Eigen::Matrix<double, 7, 7> M; //Mass matrix
        Eigen::Matrix<double, 7, 7> M_inv; //inverse of M
        Eigen::Matrix<double, 7, 1> b; //coriolis vector
        Eigen::Matrix<double, 7, 1> g; //gravity vector
        Eigen::Matrix<double, 6, 7> J; //EE-Jacobian
        Eigen::Matrix<double, 6, 7> J_last; //EE-Jacobian of last control loop iteration(to calculate derivative)
        Eigen::Matrix<double, 7, 6> J_T; //transposed Jacobian
    };

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServer;
    //void action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal, ActionServer* server);

    class ForceActionController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface> {
    public:
        ForceActionController() :
            node_handle_("force_action_controller"),
            action_server_(node_handle_, "follow_joint_trajectory",boost::bind(&ForceActionController::action_callback, this, _1, &action_server_), false)
        {
            action_server_.start();
        }

        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal, ActionServer* server);
        void update_state();
        Eigen::Matrix<double, 6, 1> get_pose_error();
        void get_dynamic_model_parameters();
        void compute_task_space_matrices();
        Eigen::Matrix<double, 7, 1> add_nullspace_torque(Eigen::Matrix<double, 7, 7> Kp, Eigen::Matrix<double, 7, 7> Kd);
        void check_movement_and_log(bool log);

        /* ToDO: Setup Action Server for FollowJoinTrajectory Action to generate reference goals for the controller */
        Eigen::Matrix<double, 6, 6> Lambda; //Lambda, task space dynamics Matrix
        Eigen::Matrix<double, 6, 6> Sm = Eigen::MatrixXd::Identity(6,6); //task space selection matrix for positions and rotation
        Eigen::Matrix<double, 6, 6> Sf = Eigen::MatrixXd::Zero(6,6); ; //task space selection matrix for forces
        Eigen::Matrix<double, 6, 1> mu; //mu vector for dynamic task formulation
        Eigen::Matrix<double, 6, 1> p; //p vector for dynamic task formulation
        Eigen::Matrix<double, 7, 7> M; //Mass matrix
        Eigen::Matrix<double, 7, 7> M_inv; //inverse of M
        Eigen::Matrix<double, 7, 1> b; //coriolis vector
        Eigen::Matrix<double, 7, 1> g; //gravity vector
        Eigen::Matrix<double, 6, 7> J; //EE-Jacobian
        Eigen::Matrix<double, 7, 1> q_dot; // Joint velocities
        Eigen::Matrix<double, 7, 1> q; // Joint Positions
        Eigen::Matrix<double, 6, 7> J_last; //EE-Jacobian of last control loop iteration(to calculate derivative)
        Eigen::Matrix<double, 7, 6> J_T; //transposed Jacobian
        Eigen::MatrixXd pinv_J_T;
        Eigen::MatrixXd pinv_J;
        Eigen::Matrix<double, 6, 7> dJ = Eigen::MatrixXd::Zero(6,7); //derivative of Jacobian
        Eigen::Matrix<double, 7, 1> tau_des; //desired joint torques (will be commanded)
        Eigen::Affine3d current_pose; //current EE-pose
        Eigen::Quaterniond orientation;
        Eigen::Vector3d position;
        Eigen::Matrix<double, 6, 1> w; //velocity of EE

        Eigen::Matrix<double, 6, 1>  F_contact_des = Eigen::MatrixXd::Zero(6,6); //desired contact force
        Eigen::Matrix<double, 6, 1>  w_dot_des = Eigen::MatrixXd::Zero(6,6); //desired
        Eigen::Affine3d pose_desired; //desired pose
        Eigen::Quaterniond orientation_desired;
        Eigen::Vector3d translation_desired;

        Eigen::Matrix<double, 6, 1> w_des = Eigen::MatrixXd::Zero(6,6); //desired velocity of EE
        Eigen::Matrix<double, 6, 1> F_last = Eigen::MatrixXd::Zero(6,1);; //previous F value
        Eigen::Matrix<double, 6, 1> dF_last = Eigen::MatrixXd::Zero(6,1);; //previous F value

        long int count = 0;
    private:
        // Saturation
        ros::NodeHandle node_handle_;
        ActionServer action_server_;
        franka_hw::FrankaStateInterface* franka_state_interface_;
        franka_hw::FrankaModelInterface* model_interface_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        franka::RobotState robot_state_;

        Eigen::Matrix<double, 7, 1> saturateTorqueRate(
                const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                const Eigen::Matrix<double, 7, 1>& tau_J_d);


        static constexpr double kDeltaTauMax{50};

        Eigen::Matrix<double, 7, 1> k_gains_;
        Eigen::Matrix<double, 7, 1> d_gains_;
        std::vector<double> i_gains_;
        Eigen::Matrix<double, 7, 1> tau_bias = Eigen::MatrixXd::Zero(7,1);
        Eigen::Matrix<double, 6, 1> F_bias = Eigen::MatrixXd::Zero(6,1);
        Eigen::Matrix<double, 6, 1> delta_F = Eigen::MatrixXd::Zero(6,1);
        Eigen::Matrix<double, 6, 1> error_F = Eigen::MatrixXd::Zero(6,1);
        Eigen::Matrix<double, 7, 1> Tau_err = Eigen::MatrixXd::Zero(7,1);
        std::array<double, 7> dq_filtered_;
        Eigen::Matrix<double, 7, 1> dq_desired = Eigen::MatrixXd::Zero(7,1);
        Eigen::Matrix<double, 7, 1> q_desired = Eigen::MatrixXd::Zero(7,1);
        std::array<double, 16> initial_pose_;
        Eigen::Matrix<double, 6, 1> I_error = Eigen::MatrixXd::Zero(6,1);
        std::array<double, 16> F_T_EE; //end effector in flange frame
        std::array<double, 16> EE_T_K; //stiffness frame in EE frame
        double additional_task = 1.0;
        bool direction_changed = false;
        double alpha = 0.001; //filter rate for F
        double beta = 0.01; //filter rate for dF
        const double dt = 0.001; //time between controller updates
        franka_hw::TriggerRate rate_trigger_{1/dt};
        State current_state;





    };

}  // namespace force_control
// force_control

#endif //FORCE_ACTION_CONTROLLER_H

