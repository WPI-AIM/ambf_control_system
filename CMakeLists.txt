cmake_minimum_required(VERSION 3.0.0)
project(ambf_control_system VERSION 2.0)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out/bin)

add_subdirectory(Sway)
# add_subdirectory(Sandbox)
add_subdirectory(Tests)