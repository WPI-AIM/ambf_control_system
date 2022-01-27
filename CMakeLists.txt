cmake_minimum_required(VERSION 3.0.2)
project(ambf_control_system)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic/")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
include_directories (${PROJECT_SOURCE_DIR}/external/Eigen)
# include_directories (${PROJECT_SOURCE_DIR}/external/Eigen)


# static library external dependencies source files
file (GLOB_RECURSE source_eigen     RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/Eigen/Eigen/*)
# file (GLOB_RECURSE source_eigen     RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/Eigen/Eigen/*)

# group source files (MSVC likes this)
foreach (FILE ${source} ${source_eigen})
  get_filename_component (PARENT_DIR "${FILE}" PATH)
  string (REGEX REPLACE "(\\./)?(src|include)/?" "" GROUP "${PARENT_DIR}")
  string (REPLACE "/" "\\" GROUP "${GROUP}")
  source_group ("${GROUP}" FILES "${FILE}")
endforeach ()

# add_library(eigen3 STATIC ${source_eigen} )

# # AMBF library exports
# set (Eigen_INCLUDE_DIRS
#     "${PROJECT_SOURCE_DIR}/external/Eigen"
#     "${Boost_INCLUDE_DIRS}")


# output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ambf_control_system)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rbdl_model)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rbdl_model_tests)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rbdl_server)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rbdl_model_test)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/trajectory_generator)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/controller_modules)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/controller_modules_test)
