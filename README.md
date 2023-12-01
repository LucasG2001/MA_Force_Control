# MA_Force_Control
force controller package for master thesis

The working controller is the "cartesian impedance controller" which implements a hybrid impedance/force control scheme.
You can either control the robot by a marker node, goal_state_publisher package from me or with moveit. At the moment the subscription to the marker node and equilibrium pose is commented out, so switch it around if needed. At the moment it is not possible to have multiple at the same time. 

Build:
Check the dependencies in the CmakeLists.txt 
In particular, you need to have Moveit1 fully installed and ready in the same workspace you want to use this package as well as:
-libfranka
-franka_ros

Launch Files:
-cartesian_impedance_controller.launch -> starts the controller on the panda or fr3 robot with RVIZ and Moveit Interface (note it will still not receive positions from moveit if the node is commented out)
-> For control: Launch desired program from goal_state_publisher package

Issues:

Note:
-To adapt force thresholds you need to change the configs in the franka control and franka_hw packages

