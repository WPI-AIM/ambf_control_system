# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rbdl_server: 0 messages, 6 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rbdl_server_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv" NAME_WE)
add_custom_target(_rbdl_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbdl_server" "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv" NAME_WE)
add_custom_target(_rbdl_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbdl_server" "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv" ""
)

get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv" NAME_WE)
add_custom_target(_rbdl_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbdl_server" "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv" ""
)

get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv" NAME_WE)
add_custom_target(_rbdl_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbdl_server" "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv" ""
)

get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv" NAME_WE)
add_custom_target(_rbdl_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbdl_server" "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv" ""
)

get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv" NAME_WE)
add_custom_target(_rbdl_server_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbdl_server" "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbdl_server
)
_generate_srv_cpp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbdl_server
)
_generate_srv_cpp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbdl_server
)
_generate_srv_cpp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbdl_server
)
_generate_srv_cpp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbdl_server
)
_generate_srv_cpp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbdl_server
)

### Generating Module File
_generate_module_cpp(rbdl_server
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbdl_server
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rbdl_server_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rbdl_server_generate_messages rbdl_server_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_cpp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_cpp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_cpp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_cpp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_cpp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_cpp _rbdl_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbdl_server_gencpp)
add_dependencies(rbdl_server_gencpp rbdl_server_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbdl_server_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbdl_server
)
_generate_srv_eus(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbdl_server
)
_generate_srv_eus(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbdl_server
)
_generate_srv_eus(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbdl_server
)
_generate_srv_eus(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbdl_server
)
_generate_srv_eus(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbdl_server
)

### Generating Module File
_generate_module_eus(rbdl_server
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbdl_server
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rbdl_server_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rbdl_server_generate_messages rbdl_server_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_eus _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_eus _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_eus _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_eus _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_eus _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_eus _rbdl_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbdl_server_geneus)
add_dependencies(rbdl_server_geneus rbdl_server_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbdl_server_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbdl_server
)
_generate_srv_lisp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbdl_server
)
_generate_srv_lisp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbdl_server
)
_generate_srv_lisp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbdl_server
)
_generate_srv_lisp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbdl_server
)
_generate_srv_lisp(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbdl_server
)

### Generating Module File
_generate_module_lisp(rbdl_server
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbdl_server
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rbdl_server_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rbdl_server_generate_messages rbdl_server_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_lisp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_lisp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_lisp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_lisp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_lisp _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_lisp _rbdl_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbdl_server_genlisp)
add_dependencies(rbdl_server_genlisp rbdl_server_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbdl_server_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbdl_server
)
_generate_srv_nodejs(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbdl_server
)
_generate_srv_nodejs(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbdl_server
)
_generate_srv_nodejs(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbdl_server
)
_generate_srv_nodejs(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbdl_server
)
_generate_srv_nodejs(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbdl_server
)

### Generating Module File
_generate_module_nodejs(rbdl_server
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbdl_server
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rbdl_server_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rbdl_server_generate_messages rbdl_server_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_nodejs _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_nodejs _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_nodejs _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_nodejs _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_nodejs _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_nodejs _rbdl_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbdl_server_gennodejs)
add_dependencies(rbdl_server_gennodejs rbdl_server_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbdl_server_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server
)
_generate_srv_py(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server
)
_generate_srv_py(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server
)
_generate_srv_py(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server
)
_generate_srv_py(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server
)
_generate_srv_py(rbdl_server
  "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server
)

### Generating Module File
_generate_module_py(rbdl_server
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rbdl_server_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rbdl_server_generate_messages rbdl_server_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLKinimatics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_py _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLModel.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_py _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLInverseDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_py _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLBodyNames.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_py _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLForwardDynamics.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_py _rbdl_server_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/localcodebase/ambf_repos/ambf_control_system/rbdl_server/srv/RBDLJacobian.srv" NAME_WE)
add_dependencies(rbdl_server_generate_messages_py _rbdl_server_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbdl_server_genpy)
add_dependencies(rbdl_server_genpy rbdl_server_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbdl_server_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbdl_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbdl_server
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rbdl_server_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rbdl_server_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbdl_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbdl_server
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rbdl_server_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(rbdl_server_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbdl_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbdl_server
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rbdl_server_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rbdl_server_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbdl_server)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbdl_server
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rbdl_server_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(rbdl_server_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbdl_server
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rbdl_server_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rbdl_server_generate_messages_py geometry_msgs_generate_messages_py)
endif()
