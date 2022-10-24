project(Sway VERSION 2.0)
cmake_minimum_required(VERSION 3.0.0)

add_subdirectory(vendor/spdlog)
add_subdirectory(vendor/eigen)
add_subdirectory(vendor/Catch2)
add_subdirectory(vendor/rbdl)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

set(PCH_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/PCH/include PARENT_SCOPE)

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Catch2 REQUIRED)
# find_package(Dependent_Packages REQUIRED)
find_package(RBDL REQUIRED)
find_package(AMBF_CLIENT REQUIRED)

include_directories(
  ${spdlog_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Catch2_INCLUDE_DIRS}
  ${RBDL_CONFIG_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
  ${ROS_INCLUDE_DIRS}
  ${AMBF_CLIENT_INCLUDE_DIRS}
  ${AMBF_MSGS_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/PCH/include
  )

add_library(${PROJECT_NAME} STATIC
  src/Application.cpp 
  src/Log.cpp
)

target_precompile_headers(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/PCH/include/PCH/pch.h)

target_include_directories(${PROJECT_NAME} 
  PRIVATE include)

target_link_libraries(${PROJECT_NAME}
  PRIVATE ${spdlog_BINARY_DIR}/libspdlog.a
  PRIVATE ${AMBF_CLIENT_LIBRARIES}
)

# include(Sway.h)