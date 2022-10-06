cmake_minimum_required(VERSION 3.0.0)
project(ambf_control_system VERSION 2.0.0)

add_subdirectory(external/eigen)
add_subdirectory(external/Catch2)
add_subdirectory(external/rbdl)



# list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic/")

# find_package(catkin REQUIRED)

# output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out/bin)

add_executable(${PROJECT_NAME}_app
main.cpp)

target_include_directories(${PROJECT_NAME}_app
  PUBLIC external/eigen/
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
