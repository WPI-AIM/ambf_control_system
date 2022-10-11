project(Sway VERSION 2.0)
cmake_minimum_required(VERSION 3.0.0)

add_library(${PROJECT_NAME}
src/Application.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include)
include(Sway.h)