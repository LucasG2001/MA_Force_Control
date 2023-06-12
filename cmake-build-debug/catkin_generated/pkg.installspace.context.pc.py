# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_interface;dynamic_reconfigure;eigen_conversions;franka_hw;franka_gripper;geometry_msgs;hardware_interface;joint_limits_interface;tf;tf_conversions;message_runtime;pluginlib;realtime_tools;roscpp;urdf;visualization_msgs;actionlib;spdlog".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lforce_control;/usr/lib/libfranka.so.0.10.0".split(';') if "-lforce_control;/usr/lib/libfranka.so.0.10.0" != "" else []
PROJECT_NAME = "force_control"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.10.1"
