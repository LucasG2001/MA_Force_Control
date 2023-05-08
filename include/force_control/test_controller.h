//
// Created by lucas on 16.04.23.
//
#pragma once
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <force_control/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>


#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>


#ifndef FORCE_CONTROL_TEST_CONTROLLER_H
#define FORCE_CONTROL_TEST_CONTROLLER_H

namespace force_control {

    class TestController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            //hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void update(const ros::Time&, const ros::Duration& period) override;

    private:
        // Saturation
        franka_hw::FrankaStateInterface* franka_state_interface_;
        franka_hw::FrankaModelInterface* model_interface_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;

        static constexpr double kDeltaTauMax{1.0};
        double radius_{0.1};
        double acceleration_time_{2.0};
        double vel_max_{0.05};
        double angle_{0.0};
        double vel_current_{0.0};

        std::vector<double> k_gains_;
        std::vector<double> d_gains_;
        double coriolis_factor_{1.0};
        std::array<double, 7> dq_filtered_;
        std::array<double, 16> initial_pose_;

        franka_hw::TriggerRate rate_trigger_{1.0};
        std::array<double, 7> last_tau_d_{};
        realtime_tools::RealtimePublisher<JointTorqueComparison> torques_publisher_;
    };

}  // namespace force_control
// force_control

#endif //FORCE_CONTROL_TEST_CONTROLLER_H
