//
// Created by lucas on 30.11.23.
//

#include <ros/node_handle.h>
#include <force_control/ImpedanceParameterController.h>
#include <std_msgs/Float64.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "impedance_controller_node");
	ros::NodeHandle nh;

	ImpedanceParameterController controller;
	ros::Subscriber rightHandSub = nh.subscribe("/right_hand", 1, &ImpedanceParameterController::rightHandCallback, &controller);
	ros::Subscriber leftHandSub = nh.subscribe("/left_hand", 1, &ImpedanceParameterController::leftHandCallback, &controller);
	ros::Subscriber FextSub = nh.subscribe("/F_ext", 1, &ImpedanceParameterController::FextCallback, &controller);

	ros::Rate loop_rate(20); // 20Hz

	while (ros::ok()) {
		controller.updateImpedanceParameters();

		// Publish stiffness parameter of the active task
		std_msgs::Float64 stiffness_msg;
		// Assuming there is a method to get stiffness from the active task
		stiffness_msg.data = controller.getStiffness(); // Update with actual method
		ros::Publisher stiffness_pub = nh.advertise<std_msgs::Float64>("/stiffness", 1);
		stiffness_pub.publish(stiffness_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

