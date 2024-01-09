#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from franka_msgs.msg import FrankaState
import numpy as np


class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger_node', anonymous=True)

        self.hand_position = None
        self.ee_position = None
        self.time_count = 0

        # File to save the data
        self.file_path = "data_log.txt"

        # ROS Subscribers
        rospy.Subscriber("/cartesian_impedance_controller/right_hand", Point, self.hand_position_callback)
        # TODO: Change Topic Name to correct Topic for Franka State
        rospy.Subscriber("/franka_states", FrankaState, self.franka_states_callback)

        # Set the logging frequency
        self.logging_rate = rospy.Rate(10)  # 10 Hz

        # Main loop
        self.run()

    def hand_position_callback(self, data):
        self.hand_position = np.array([data.x, data.y, data.z])

    def franka_states_callback(self, data):
        # TODO: Adapt to access correct information
        self.ee_position = np.array(data.O_T_EE)

    def log_data(self):
        if self.hand_position is not None and self.ee_position is not None:
            data_to_log = np.concatenate((self.hand_position, self.ee_position, [self.time_count]))

            with open(self.file_path, 'a') as file:
                np.savetxt(file, [data_to_log], delimiter=',', fmt='%.6f')

    def run(self):
        while not rospy.is_shutdown():
            self.log_data()
            self.time_count += 1
            self.logging_rate.sleep()


if __name__ == '__main__':
    try:
        data_logger = DataLogger()
    except rospy.ROSInterruptException:
        pass
