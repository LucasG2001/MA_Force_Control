//
// Created by lucas on 01.12.23.
//
#include "force_control/ActionPrimitive.h"  // Update with your actual package name

// Constructor
ActionPrimitive::ActionPrimitive()
		: grasp_(false),
		  spring_stiffness_(0.0),
		  damping_(0.0),
		  inertia_(0.0),
		  repulsion_stiffness_(0.0),
		  repulsion_damping_(0.0) {
	// Initialize other members if needed
}

// Setters
void ActionPrimitive::setStartPose(const Eigen::Matrix<double, 6, 1>& start_pose) {
	start_pose_ = start_pose;
}

void ActionPrimitive::setGoalPose(const Eigen::Matrix<double, 6, 1>& goal_pose) {
	goal_pose_ = goal_pose;
}

void ActionPrimitive::setObjectPose(const Eigen::Matrix<double, 6, 1>& object_pose) {
	object_pose_ = object_pose;
}

void ActionPrimitive::setGrasp(bool grasp) {
	grasp_ = grasp;
}

void ActionPrimitive::setParameters(double spring_stiffness, double damping, double inertia,
                                    double repulsion_stiffness, double repulsion_damping) {
	spring_stiffness_ = spring_stiffness;
	damping_ = damping;
	inertia_ = inertia;
	repulsion_stiffness_ = repulsion_stiffness;
	repulsion_damping_ = repulsion_damping;
}

// Getters
Eigen::Matrix<double, 6, 1> ActionPrimitive::getStartPose() const {
	return start_pose_;
}

Eigen::Matrix<double, 6, 1> ActionPrimitive::getGoalPose() const {
	return goal_pose_;
}

Eigen::Matrix<double, 6, 1> ActionPrimitive::getObjectPose() const {
	return object_pose_;
}

bool ActionPrimitive::getGrasp() const {
	return grasp_;
}

double ActionPrimitive::getSpringStiffness() const {
	return spring_stiffness_;
}

double ActionPrimitive::getDamping() const {
	return damping_;
}

double ActionPrimitive::getInertia() const {
	return inertia_;
}

double ActionPrimitive::getRepulsionStiffness() const {
	return repulsion_stiffness_;
}

double ActionPrimitive::getRepulsionDamping() const {
	return repulsion_damping_;
}
