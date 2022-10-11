cmake_minimum_required(VERSION 3.0.0)
project(Sandbox VERSION 2.0)


add_executable(${PROJECT_NAME}_app
src/SandboxApp.cpp)

target_include_directories(${PROJECT_NAME}_app PRIVATE ../Sway)

target_link_libraries(${PROJECT_NAME}_app PRIVATE Sway)