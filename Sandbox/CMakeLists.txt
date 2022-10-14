cmake_minimum_required(VERSION 3.0.0)
project(Sandbox VERSION 2.0)

add_executable(${PROJECT_NAME}_app
src/SandboxApp.cpp)

target_include_directories(${PROJECT_NAME}_app 
PRIVATE ${Sway_SOURCE_DIR}
PRIVATE ${SPDLOG_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_app 
PRIVATE ${Sway_BINARY_DIR}/libSway.a
PRIVATE ${spdlog_BINARY_DIR}/libspdlog.a)