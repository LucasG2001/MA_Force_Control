# MA_Force_Control
force controller package for master thesis

The working controller is the "cartesian impedance controller" which implements a hybrid impedance/force control scheme.
You can either control the robot by a marker node or with moveit. At the moment the subscription to the marker node is commented out, so switch it around if needed. At the moment it is not possible to have both at the same time. 

Launch Files:
-cartesian_impedance_controller.launch -> starts the controller on the panda or fr3 robot
- for moveit control run panda/fr3_moveit_config move_group.launch separately

Issues:
- There are some issues with the gripper, especially when starting the robot as fr3, it doesn't react to all commands.
  Reason is unknown yet.
