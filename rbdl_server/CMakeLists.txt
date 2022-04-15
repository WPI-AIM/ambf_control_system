cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(rbdl_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

get_filename_component(PARENT_CMAKE ../cmake ABSOLUTE)
set (CMAKE_MODULE_PATH ${PARENT_CMAKE} )
# message(STATUS "foo include dir: " ${PARENT_CMAKE} )
FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
FIND_PACKAGE (yaml-cpp REQUIRED)
#FIND_PACKAGE (RBDL REQUIRED)
#find_package(RBDL REQUIRED PATHS "/home/shreyas/rbdl/examples/simple/")

find_package(catkin REQUIRED COMPONENTS
roscpp
geometry_msgs
tf
message_generation
rbdl_model
)


add_service_files(
  FILES
  RBDLModel.srv
  RBDLForwardDynamics.srv
  RBDLInverseDynamics.srv
  RBDLJacobian.srv
  RBDLKinimatics.srv
  RBDLInverseKinimatics.srv
  RBDLBodyNames.srv
  RBDLModelAlignment.srv
  RBDLJointSpaceInertia.srv
  RBDLPointVelocity.srv
  RBDLNonlinearEffects.srv
  RBDLTaskSpaceBody.srv
  
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rbdl_server rbdl_model
 CATKIN_DEPENDS roscpp message_runtime
 # DEPENDS system_lib
  DEPENDS Eigen
  DEPENDS rbdl_model
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${rbdl_model_INCLUDE_DIRS})

include_directories( include ${RBDL_MODEL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})


find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})



FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
include_directories(include ${RBDL_MODEL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${RBDL_INCLUDE_DIR})
include_directories(RBDL_MODEL_INCLUDE_DIRS)

set(RBDL_HRD_FILES
  include/${PROJECT_NAME}/RBDLServer.h

)

set(RBDL_SRC_FILES
  src/RBDLServer.cpp

)

## Declare a C++ library
add_library(${PROJECT_NAME} ${RBDL_SRC_FILES} ${RBDL_HRD_FILES})


add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES} ${Eigen_LIBRARIES} ${RBDL_LIBRARY} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${RBDL_MODEL_LIBRARIES} rbdl_model )

set (RBDL_SERVER_INCLUDE_DIRS include)
set (RBDL_SERVER_LIBRARIES rbdl_server)


install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


set(SOURCE_FILES 
    src/rbdl_server_node.cpp)

add_executable (rbdl_server_node ${SOURCE_FILES})
target_link_libraries (rbdl_server_node ${PROJECT_NAME})


add_executable (rbdl_main src/main.cpp)
target_link_libraries (rbdl_main ${PROJECT_NAME})
