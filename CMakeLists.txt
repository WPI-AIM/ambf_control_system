cmake_minimum_required(VERSION 3.0.0)
project(ambf_control_system VERSION 2.0.0)

add_subdirectory(external/eigen)
add_subdirectory(external/Catch2)
add_subdirectory(external/rbdl)
# add_subdirectory(tests)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)
find_package(CATCH2 REQUIRED)
find_package(RBDL REQUIRED)

# list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic/")

# find_package(catkin REQUIRED)


# output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out/bin)

add_executable(${PROJECT_NAME}_app
main.cpp)

message("RBDL_INCLUDE_DIRS" ${RBDL_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_app
PUBLIC ${CMAKE_SOURCE_DIR}/external/eigen/
PUBLIC ${CATCH2_INCLUDE_DIRS}
PUBLIC ${RBDL_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME}_app 
  PRIVATE ${RBDL_DIR}
)


target_link_libraries(${PROJECT_NAME}_app 
PRIVATE rbdl # Works, but need a better approach
# PRIVATE ${PROJECT_SOURCE_DIR}/out/build/release/external/rbdl/librbdl.so.3.1.3
)

set_target_properties(${PROJECT_NAME}_app PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)


add_subdirectory(tests)