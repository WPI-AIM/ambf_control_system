project(Sway VERSION 2.0)
cmake_minimum_required(VERSION 3.0.0)

add_subdirectory(vendor/spdlog)

set(SPDLOG_INCLUDE_DIRS ${spdlog_SOURCE_DIR}/include PARENT_SCOPE)
SET(PCH_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/PCH/include PARENT_SCOPE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/PCH/include)

add_library(${PROJECT_NAME} STATIC
  src/Application.cpp 
  src/Log.cpp
)

target_precompile_headers(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/PCH/include/PCH/pch.h)

target_include_directories(${PROJECT_NAME}
  PRIVATE ${spdlog_SOURCE_DIR}/include
  PRIVATE include
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE ${spdlog_BINARY_DIR}/libspdlog.a
)

include(Sway.h)