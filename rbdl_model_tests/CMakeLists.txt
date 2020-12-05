cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(rbdl_model_tests)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  #ambf_msgs
  geometry_msgs
  message_generation
  tf
  rbdl_model
  std_msgs
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
 #ambf_msgs
)


catkin_package( INCLUDE_DIRS include
  LIBRARIES rbdl_model_tests rbdl_model
  CATKIN_DEPENDS
  #ambf_msgs
  geometry_msgs message_runtime std_msgs
  DEPENDS rbdl_model
)

FIND_PACKAGE (yaml-cpp REQUIRED)
#FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
include_directories(include ${YAML_CPP_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS} ${RBDL_INCLUDE_DIR})

include_directories( include ${catkin_INCLUDE_DIRS})
include_directories(RBDL_MODEL_INCLUDE_DIRS)

# Needed for UnitTest++
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake )

# Look for unittest++
FIND_PACKAGE (UnitTest++ REQUIRED)
INCLUDE_DIRECTORIES (${UNITTEST++_INCLUDE_DIR})

#FIND_PACKAGE (ambf_client REQUIRED)
#include_directories(${AMBF_CLIENT_INCLUDE_DIR})
#MESSAGE(STATUS "Found AMBF: ${AMBF_CLIENT_INCLUDE_DIR}")

#FIND_PACKAGE (ambf_msgs REQUIRED)
#include_directories(${ambf_msgs_INCLUDE_DIR})
#MESSAGE(STATUS "Found AMBF MSGS: ${ambf_msgs_INCLUDE_DIR}")


find_package (Boost COMPONENTS program_options filesystem REQUIRED)

include_directories(${rbdl_model_INCLUDE_DIRS}  ${AMBF_CLIENT_INCLUDE_DIRS})
include_directories( include ${catkin_INCLUDE_DIRS} ${RBDL_MODEL_INCLUDE_DIRS})


set (HDR_FILES
    include/${PROJECT_NAME}/rbdl_model_test.h)

set(SOURCE_FILES
    src/rbdl_model_test.cpp
    #src/main.cpp
    )


set (RBDL_MODEL_TESTS_HDR_FILES
#    include/${PROJECT_NAME}/RBDLBodyTests.h
    include/${PROJECT_NAME}/RBDLTestPrep.h
    )

set(RBDL_MODEL_TESTS_SOURCE_FILES
    src/RBDLBodyTests.cpp
    src/RBDLJointTests.cpp
    src/RBDLTestPrep.cpp
    src/main.cpp
    )

find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

#add_executable (${PROJECT_NAME} ${HDR_FILES} ${SOURCE_FILES})
#target_link_libraries (${PROJECT_NAME} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${RBDL_LIBRARY} ${YAML_CPP_LIBRARIES} pthread rbdl_model ${RBDL_MODEL_LIBRARIES})

#add_executable (psm_tests ${PSM_TESTS_HDR_FILES} ${PSM_TESTS_SOURCE_FILES})
#target_link_libraries (psm_tests ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${RBDL_LIBRARY} ${YAML_CPP_LIBRARIES} pthread rbdl_model ${RBDL_MODEL_LIBRARIES} )


SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
  LINKER_LANGUAGE CXX
  OUTPUT_NAME rbdl_model_tests
  )

ADD_EXECUTABLE ( rbdl_model_tests ${RBDL_MODEL_TESTS_HDR_FILES} ${RBDL_MODEL_TESTS_SOURCE_FILES} )

SET_TARGET_PROPERTIES ( rbdl_model_tests PROPERTIES
  LINKER_LANGUAGE CXX
  OUTPUT_NAME rbdl_model_tests
  )

SET (RBDL_LIBRARY rbdl)
IF (RBDL_BUILD_STATIC)
  SET (RBDL_LIBRARY rbdl-static)
ENDIF (RBDL_BUILD_STATIC)

TARGET_LINK_LIBRARIES ( rbdl_model_tests
# ${AMBF_CLIENT_LIBRARY}
  ${GLFW_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${RBDL_LIBRARY}
  ${YAML_CPP_LIBRARIES}
  pthread
  rbdl_model
  ${UNITTEST++_LIBRARY}
  ${RBDL_LIBRARY}
  )


