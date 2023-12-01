//
// Created by lucas on 01.12.23.
//
#ifndef IMPEDANCE_PARAMETER_CONTROLLER_H
#define IMPEDANCE_PARAMETER_CONTROLLER_H
#endif // IMPEDANCE_PARAMETER_CONTROLLER_H

#include <force_control/AtomicTasks.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>

class ImpedanceParameterController {
public:
	ImpedanceParameterController();

	// Callback functions
	void rightHandCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void leftHandCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void FextCallback(const geometry_msgs::Pose::ConstPtr& msg);

	// Setters for active task
	void setActiveTask(ActionPrimitive& desired_task);

	// Call this method to update the impedance parameters based on the active task
	void updateImpedanceParameters();

	//dummy function
	//ToDo: implement fully
	double getStiffness();

private:
	GetMe get_me_task;
	FollowMe follow_me_task;
	HoldThis hold_this_task;
	TakeThis take_this_task;
	AvoidMe avoid_me_task;
	/* Add other actions here */

	ActionPrimitive* activeTask;
	Eigen::Matrix<double, 6, 1> rightHandPose;
	Eigen::Matrix<double, 6, 1> leftHandPose;
	Eigen::Matrix<double, 6, 1> externalForce;
};



