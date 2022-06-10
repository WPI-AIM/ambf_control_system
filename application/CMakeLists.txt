cmake_minimum_required(VERSION 3.0.2)
project(application)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic/")

get_filename_component(PARENT_CMAKE ../cmake ABSOLUTE)
set (CMAKE_MODULE_PATH ${PARENT_CMAKE} )

# set (RBDL_MODEL_LIBRARY "/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf_control_system/build/devel/lib/librbdl_model.so")
LIST( APPEND CMAKE_MODULE_PATH ${PARENT_DIR} )
find_package(RBDL REQUIRED)
# include_directories(${RBDL_INCLUDE_DIRS})


# FIND_PACKAGE (rbdl_model REQUIRED)
# include_directories(${RBDL_MODEL})
# include_directories(rbdl_model)

set (ambf_msgs_DIR ${CMAKE_MODULE_PATH})

set (AMBF_CLIENT_LIBRARY "/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf/build/devel/lib/libambf_client_cpp.so")

FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
#find_package(GTest REQUIRED)

include_directories("/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf/ambf_ros_modules/ambf_client/include/")
FIND_PACKAGE (ambf_client REQUIRED)
include_directories(${AMBF_CLIENT_INCLUDE_DIRS})


find_package(catkin REQUIRED COMPONENTS
   ambf_msgs
   geometry_msgs
   message_generation
   tf
)


find_package (Boost COMPONENTS program_options filesystem REQUIRED)
catkin_package(INCLUDE_DIRS include ${AMBF_CLIENT_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS ambf_msgs
#  DEPENDS system_lib
)


catkin_package( INCLUDE_DIRS include
  LIBRARIES 
  # rbdl_model
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)


# Look for unittest++
# FIND_PACKAGE (UnitTest++ REQUIRED)
# INCLUDE_DIRECTORIES (
#   ${AMBF_CLIENT_INCLUDE_DIRS} 
#   ${RBDL_INCLUDE_DIRS}
#   )

include_directories( include 
  ${catkin_INCLUDE_DIRS} 
  ${AMBF_CLIENT_INCLUDE_DIRS} 

  ${Eigen_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR} 
  ${RBDL_INCLUDE_DIR})


#find_package (ambf_client REQUIRED)
# find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AMBF_CLIENT_INCLUDE_DIRS} 
)

# find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
# include_directories (${GLFW_INCLUDE_DIRS})

set (APPLICATION_HDR_FILES
    include/${PROJECT_NAME}/Utilities.h
    include/${PROJECT_NAME}/RBDLModelErrors.h
    include/${PROJECT_NAME}/Prep.h    
    )

set(APPLICATION_SOURCE_FILES
    src/Utilities.cpp
    src/RBDLModelErrors.cpp
    src/Prep.cpp
    # src/main.cpp
    )



## Declare a C++ library
add_library(application ${APPLICATION_HDR_FILES} ${APPLICATION_SOURCE_FILES})

add_dependencies(application ${catkin_EXPORTED_TARGETS})

# add_executable (application
# ${APPLICATION_HDR_FILES}  
# ${APPLICATION_SOURCE_FILES})

add_library(ambf_client_lib STATIC 
"/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf/ambf_ros_modules/ambf_client/src/ambf_client.cpp"
)


target_include_directories(ambf_client_lib PUBLIC 
"/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf/ambf_ros_modules/ambf_client/include/"
)


target_link_libraries(application 
  # ambf_client_lib
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${AMBF_CLIENT_LIBRARY} 
  ${Eigen_LIBRARIES}
  ${RBDL_LIBRARY} 
  ${YAML_CPP_LIBRARIES} 
  pthread
  )

set_target_properties(application PROPERTIES
 CXX_STANDARD 17
 CXX_STANDARD_REQUIRED YES
 )

set (APPLICATION_INCLUDE_DIRS include)
set (APPLICATION_LIBRARIES application)