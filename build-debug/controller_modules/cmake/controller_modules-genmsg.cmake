# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "controller_modules: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(controller_modules_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv" NAME_WE)
add_custom_target(_controller_modules_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller_modules" "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv" ""
)

get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv" NAME_WE)
add_custom_target(_controller_modules_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controller_modules" "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv" "trajectory_msgs/JointTrajectoryPoint:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_modules
)
_generate_srv_cpp(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_modules
)

### Generating Module File
_generate_module_cpp(controller_modules
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_modules
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(controller_modules_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(controller_modules_generate_messages controller_modules_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_cpp _controller_modules_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_cpp _controller_modules_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_modules_gencpp)
add_dependencies(controller_modules_gencpp controller_modules_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_modules_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_modules
)
_generate_srv_eus(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_modules
)

### Generating Module File
_generate_module_eus(controller_modules
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_modules
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(controller_modules_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(controller_modules_generate_messages controller_modules_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_eus _controller_modules_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_eus _controller_modules_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_modules_geneus)
add_dependencies(controller_modules_geneus controller_modules_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_modules_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_modules
)
_generate_srv_lisp(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_modules
)

### Generating Module File
_generate_module_lisp(controller_modules
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_modules
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(controller_modules_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(controller_modules_generate_messages controller_modules_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_lisp _controller_modules_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_lisp _controller_modules_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_modules_genlisp)
add_dependencies(controller_modules_genlisp controller_modules_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_modules_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_modules
)
_generate_srv_nodejs(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_modules
)

### Generating Module File
_generate_module_nodejs(controller_modules
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_modules
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(controller_modules_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(controller_modules_generate_messages controller_modules_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_nodejs _controller_modules_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_nodejs _controller_modules_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_modules_gennodejs)
add_dependencies(controller_modules_gennodejs controller_modules_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_modules_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_modules
)
_generate_srv_py(controller_modules
  "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_modules
)

### Generating Module File
_generate_module_py(controller_modules
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_modules
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(controller_modules_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(controller_modules_generate_messages controller_modules_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/ControllerList.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_py _controller_modules_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/controller_modules/srv/JointControl.srv" NAME_WE)
add_dependencies(controller_modules_generate_messages_py _controller_modules_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controller_modules_genpy)
add_dependencies(controller_modules_genpy controller_modules_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controller_modules_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_modules)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controller_modules
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(controller_modules_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(controller_modules_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(controller_modules_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_modules)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controller_modules
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(controller_modules_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(controller_modules_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(controller_modules_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_modules)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controller_modules
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(controller_modules_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(controller_modules_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(controller_modules_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_modules)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controller_modules
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(controller_modules_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(controller_modules_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(controller_modules_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_modules)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_modules\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controller_modules
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(controller_modules_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(controller_modules_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(controller_modules_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
