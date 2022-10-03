cmake_minimum_required(VERSION 3.0.0)
project(ambf_control_system VERSION 2.0.0)

# list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic/")

# find_package(catkin REQUIRED)

# include_directories (${PROJECT_SOURCE_DIR}/external/Eigen)

# # static library external dependencies source files
# file (GLOB_RECURSE source_eigen     RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/Eigen/Eigen/*)

# # group source files (MSVC likes this)
# foreach (FILE ${source} ${source_eigen})
#   get_filename_component (PARENT_DIR "${FILE}" PATH)
#   string (REGEX REPLACE "(\\./)?(src|include)/?" "" GROUP "${PARENT_DIR}")
#   string (REPLACE "/" "\\" GROUP "${GROUP}")
#   source_group ("${GROUP}" FILES "${FILE}")
# endforeach ()


# output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out/bin)

add_executable(${PROJECT_NAME}_app
main.cpp)

target_include_directories(${PROJECT_NAME}_app
  PUBLIC external/rbdl/include
  )

target_link_directories(${PROJECT_NAME}_app 
  PRIVATE out/build/external/rbdl)

target_link_libraries(${PROJECT_NAME}_app 
  PRIVATE rbdl)


set_target_properties(${PROJECT_NAME}_app PROPERTIES
 CXX_STANDARD 20
 CXX_STANDARD_REQUIRED YES
 )

 add_subdirectory(external/rbdl)