//
// Created by lucas on 16.04.23.
//

#include <force_control/test_controller.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <memory>
#include <iostream>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>


#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>



namespace force_control {
    namespace {
        template <class T, size_t N>
        std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
            ostream << "[";
            std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
            std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
            ostream << "]";
            return ostream;
        }
    }  // anonymous namespace

    bool TestController::init(hardware_interface::RobotHW* robot_hw,
                                      ros::NodeHandle& node_handle) {
        franka_state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (franka_state_interface_ == nullptr) {
            ROS_ERROR("ModelExampleController: Could not get Franka state interface from hardware");
            return false;
        }
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("ModelExampleController: Could not read parameter arm_id");
            return false;
        }
        model_interface_ = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface_ == nullptr) {
            ROS_ERROR_STREAM("ModelExampleController: Error getting model interface from hardware");
            return false;
        }

        try {
            franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                    franka_state_interface_->getHandle(arm_id + "_robot"));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                    "ModelExampleController: Exception getting franka state handle: " << ex.what());
            return false;
        }

        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                    model_interface_->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                    "ModelExampleController: Exception getting model handle from interface: " << ex.what());
            return false;
        }
        return true;
    }


    void TestController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
        if (rate_trigger_()) {
            std::array<double, 49> mass = model_handle_->getMass();
            std::array<double, 7> coriolis = model_handle_->getCoriolis();
            std::array<double, 7> gravity = model_handle_->getGravity();
            std::array<double, 16> pose = model_handle_->getPose(franka::Frame::kJoint4);
            std::array<double, 42> joint4_body_jacobian =
                    model_handle_->getBodyJacobian(franka::Frame::kJoint4);
            std::array<double, 42> endeffector_zero_jacobian =
                    model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

            ROS_INFO("--------------------------------------------------");
            ROS_INFO_STREAM("mass :" << mass);
            ROS_INFO_STREAM("coriolis: " <<coriolis);
            ROS_INFO_STREAM("gravity :" << gravity);
            ROS_INFO_STREAM("joint_pose :" << pose);
            ROS_INFO_STREAM("joint4_body_jacobian :" << joint4_body_jacobian);
            ROS_INFO_STREAM("joint_zero_jacobian :" << endeffector_zero_jacobian);
        }
    }

} // force_control

PLUGINLIB_EXPORT_CLASS(force_control::TestController, controller_interface::ControllerBase)