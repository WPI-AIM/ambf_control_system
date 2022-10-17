cmake_minimum_required(VERSION 3.0.0)
project(Sandbox VERSION 2.0)

add_executable(${PROJECT_NAME}_app
src/SandboxApp.cpp)

target_include_directories(${PROJECT_NAME}_app 
PRIVATE ${Sway_SOURCE_DIR}
PRIVATE ${spdlog_INCLUDE_DIRS_PS}
PRIVATE ${PCH_INCLUDE_DIRS}
# PRIVATE ${Eigen3_SOURCE_DIRS}
PRIVATE ${Eigen3_SOURCE_DIRS}
PRIVATE ${Catch2_INCLUDE_DIRS_PS}
)

target_link_libraries(${PROJECT_NAME}_app 
PRIVATE ${Sway_BINARY_DIR}/libSway.a
PRIVATE ${spdlog_BINARY_DIR}/libspdlog.a)