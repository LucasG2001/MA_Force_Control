# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "force_control: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iforce_control:/home/lucas/catkin_ws/src/force_control/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(force_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg" NAME_WE)
add_custom_target(_force_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "force_control" "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(force_control
  "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/force_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(force_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/force_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(force_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(force_control_generate_messages force_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(force_control_generate_messages_cpp _force_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(force_control_gencpp)
add_dependencies(force_control_gencpp force_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS force_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(force_control
  "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/force_control
)

### Generating Services

### Generating Module File
_generate_module_eus(force_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/force_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(force_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(force_control_generate_messages force_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(force_control_generate_messages_eus _force_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(force_control_geneus)
add_dependencies(force_control_geneus force_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS force_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(force_control
  "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/force_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(force_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/force_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(force_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(force_control_generate_messages force_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(force_control_generate_messages_lisp _force_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(force_control_genlisp)
add_dependencies(force_control_genlisp force_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS force_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(force_control
  "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/force_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(force_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/force_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(force_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(force_control_generate_messages force_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(force_control_generate_messages_nodejs _force_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(force_control_gennodejs)
add_dependencies(force_control_gennodejs force_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS force_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(force_control
  "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/force_control
)

### Generating Services

### Generating Module File
_generate_module_py(force_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/force_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(force_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(force_control_generate_messages force_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lucas/catkin_ws/src/force_control/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(force_control_generate_messages_py _force_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(force_control_genpy)
add_dependencies(force_control_genpy force_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS force_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/force_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/force_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/force_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/force_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/force_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/force_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/force_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/force_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/force_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/force_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/force_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
