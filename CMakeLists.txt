cmake_minimum_required(VERSION 3.0.0)
project(ambf_control_system VERSION 2.0)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out/bin)

add_subdirectory(Sway)
add_subdirectory(Sandbox)
# message("CMAKE_PROJECT_NAME.h.in: " ${CMAKE_PROJECT_NAME}.h.in)

# configure_file(${CMAKE_PROJECT_NAME}.h.in ${CMAKE_PROJECT_NAME}.h)

# add_subdirectory(external/eigen)
# add_subdirectory(external/Catch2)
# add_subdirectory(external/rbdl)
# add_subdirectory(external/spdlog)

# list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)
# find_package(CATCH2 REQUIRED)
# find_package(RBDL REQUIRED)
# find_package(AMBF_CLIENT REQUIRED)


# SET(SPD_INCLUDE_DIRS external/spdlog/inclsudo apt install snapdude/)
# SET(PCH_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/PCH/include)
# include_directories(
#   ${Eigen3_SOURCE_DIR}
#   ${CATCH2_INCLUDE_DIRS}
#   ${RBDL_INCLUDE_DIRS}
#   ${PCH_INCLUDE_DIRS}
#   ${AMBF_CLIENT_INCLUDE_DIRS}
#   ${SPD_INCLUDE_DIRS}
# )

# # list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic/")

# # find_package(catkin REQUIRED)

# output location

# add_executable(${PROJECT_NAME}_app
# main.cpp)

# target_precompile_headers(${PROJECT_NAME}_app PRIVATE ${PCH_INCLUDE_DIRS}/PCH/pch.h)

# message("PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR})
# target_include_directories(${PROJECT_NAME}_app PUBLIC ${PROJECT_BINARY_DIR})
# target_link_directories(${PROJECT_NAME}_app 
#   PRIVATE ${RBDL_DIR}
#   PRIVATE ${AMBF_CLIENT_LIBRARIES}
# )

# target_link_libraries(${PROJECT_NAME}_app 
# PRIVATE rbdl # Works, but need a better approach
# # PRIVATE ${PROJECT_SOURCE_DIR}/out/build/release/external/rbdl/librbdl.so.3.1.3
# PRIVATE ${AMBF_CLIENT_LIBRARIES}
# )

# set_target_properties(${PROJECT_NAME}_app PROPERTIES
# LINKER_LANGUAGE CXX
# CXX_STANDARD 20
# CXX_STANDARD_REQUIRED YES
# )

# add_subdirectory(tests)