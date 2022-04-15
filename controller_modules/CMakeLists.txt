cmake_minimum_required(VERSION 3.0.2)
project(controller_modules)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  trajectory_msgs
  message_generation
  tf
  roscpp
)

#set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
get_filename_component(PARENT_CMAKE ../cmake ABSOLUTE)
set (CMAKE_MODULE_PATH ${PARENT_CMAKE} )
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)

include_directories( include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
)

add_service_files(
  FILES
  JointControl.srv  
  ControllerList.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  trajectory_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES controller_modules
  CATKIN_DEPENDS geometry_msgs trajectory_msgs  tf
  #DEPENDS system_lib
)


set(SRC_FILES
   src/PDController.cpp
   src/ModelController.cpp
   src/SMCController.cpp
   src/GravityCompensationController.cpp
   src/ControllerManager.cpp
   src/TaskSpaceController.cpp)

set (HDR_FILES
   include/${PROJECT_NAME}/ControllerBase.h
   include/${PROJECT_NAME}/PDController.h
   include/${PROJECT_NAME}/SMCController.h
   include/${PROJECT_NAME}/ModelController.h
   include/${PROJECT_NAME}/GravityCompensationController.h
   include/${PROJECT_NAME}/ControllerManager.h
   include/${Project_NAME}/TaskSpaceController.h)

# target_link_libraries (PDController  )

## Declare a C++ library
add_library(${PROJECT_NAME} ${SRC_FILES} ${HRD_FILES} )
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${catkin_LIBRARIES})


find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

# add_executable (control_node src/control_node.cpp)
# target_link_libraries (control_node ${PROJECT_NAME} ${Eigen_LIBRARIES} ambf_client)



#add_executable (PDcontroller_test src/PDtesting_controller.cpp)
#target_link_libraries (PDcontroller_test ${PROJECT_NAME} ${GTEST_LIBRARIES} ${GLFW_LIBRARIES} pthread )

#add_executable (controlnode_test src/controlnode_testing.cpp)
#target_link_libraries (controlnode_test ${PROJECT_NAME} ${AMBF_LIBRARIES} ${GTEST_LIBRARIES} ${GLFW_LIBRARIES} pthread ambf_client )

set (CONTROLLER_MODULES_INCLUDE_DIRS include)
set (CONTROLLER_MODULES_LIBRARIES ${PROJECT_NAME})

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
