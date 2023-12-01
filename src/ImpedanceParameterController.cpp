//
// Created by lucas on 01.12.23.
//
#include <force_control/ImpedanceParameterController.h>


ImpedanceParameterController::ImpedanceParameterController()
		: get_me_task(), follow_me_task(), hold_this_task(), take_this_task(), avoid_me_task(),
		  activeTask(&hold_this_task), rightHandPose(), leftHandPose(), externalForce() {
	// Initialize other members if needed
}

void ImpedanceParameterController::rightHandCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the right hand pose
	Eigen::Vector3d right_hand_position(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Vector3d right_hand_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
	rightHandPose << right_hand_position, right_hand_orientation;
}

void ImpedanceParameterController::leftHandCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the left hand pose
	Eigen::Vector3d left_hand_position(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Vector3d left_hand_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
	leftHandPose << left_hand_position, left_hand_orientation;
}

void ImpedanceParameterController::FextCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the external force
	Eigen::Vector3d F_ext_position(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Vector3d F_ext_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
	externalForce << F_ext_position, F_ext_orientation;
}

void ImpedanceParameterController::setActiveTask(ActionPrimitive& desired_task) {
	this->activeTask = &desired_task;
}

double ImpedanceParameterController::getStiffness() {
	double k = this->activeTask->getSpringStiffness();
	return k;
}

void ImpedanceParameterController::updateImpedanceParameters() {
	//ToDo: Implement if useful here
}
