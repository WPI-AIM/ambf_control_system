cmake_minimum_required(VERSION 3.0.2)
project(rbdl_model_tests)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic/")

get_filename_component(PARENT_CMAKE ../cmake ABSOLUTE)
set (CMAKE_MODULE_PATH ${PARENT_CMAKE} )

set (RBDL_MODEL_LIBRARY "/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf_control_system/build/devel/lib/librbdl_model.so")
LIST( APPEND CMAKE_MODULE_PATH ${PARENT_DIR} )
find_package(RBDL REQUIRED)
include_directories(${RBDL_INCLUDE_DIRS})

include_directories("/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf_control_system/rbdl_model/include/")

FIND_PACKAGE (rbdl_model REQUIRED)
include_directories(${RBDL_MODEL})
include_directories(rbdl_model)

# set (ambf_msgs_DIR ${CMAKE_MODULE_PATH})
set (ambf_msgs_DIR /mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf_control_system/cmake/ )

set (AMBF_CLIENT_LIBRARY "/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf/build/devel/lib/libambf_client_cpp.so")

FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
#find_package(GTest REQUIRED)

include_directories("/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf/ambf_ros_modules/ambf_client/include/")
FIND_PACKAGE (ambf_client REQUIRED)
include_directories(${AMBF_CLIENT_INCLUDE_DIR})

set (application_DIR "/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf_control_system/cmake/")
include_directories("/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf_control_system/application/include/")
FIND_PACKAGE (application REQUIRED)
# MESSAGE(STATUS "Found Application: ${APPLICATION_INCLUDE_DIR}")
set (RBDL_MODEL_LIBRARY "/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf_control_system/build/devel/lib/librbdl_model.so")
set (APPLICATION_LIBRARY "/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf_control_system/build/devel/lib/libapplication.so")

include_directories(${AMBF_CLIENT_INCLUDE_DIRS} ${APPLICATION_INCLUDE_DIRS})
include_directories("/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf_control_system/application/include/")

FIND_PACKAGE (application REQUIRED)
include_directories(${APPLICATION})
include_directories(application)

find_package(catkin REQUIRED COMPONENTS
  ambf_msgs
  geometry_msgs
  message_generation
  tf
)


find_package (Boost COMPONENTS program_options filesystem REQUIRED)
catkin_package(INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS ambf_msgs
#  DEPENDS system_lib
)


catkin_package( INCLUDE_DIRS include
  LIBRARIES rbdl_model
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

# FIND_PACKAGE (yaml-cpp REQUIRED)
# Look for unittest++
FIND_PACKAGE (UnitTest++ REQUIRED)
INCLUDE_DIRECTORIES (${UNITTEST++_INCLUDE_DIR})
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
include_directories( include 
                    ${AMBF_CLIENT_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS} 
                    ${Eigen_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIR} 
                    ${RBDL_INCLUDE_DIR}
                    ${APPLICATION_INCLUDE_DIR}
                    ${APPLICATION_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}/application/include/
)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories (
  ${PROJECT_NAME}/include
)

# find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)


set (RBDL_MODEL_TESTS_HDR_FILES
    include/${PROJECT_NAME}/rbdl_tests.h
    )

# ################START################
# #Executible kuka_hc_tests

# set (KUKA_RBDL_MODEL_TESTS_HDR_FILES_HC
#     include/${PROJECT_NAME}/AMBFParams.h
#     include/${PROJECT_NAME}/KUKA.h
#     )

# set(KUKA_RBDL_MODEL_TESTS_SOURCE_FILES_HC
#     # src/ECMHCDynamicsTests.cc
#     src/KukaHCKinematicsTests.cc
#     # src/KUKABodyTests.cc
#     src/AMBFParams.cpp
#     src/KUKA.cpp
#     src/EigenUtilities.cpp
#     src/RBDLTestPrep.cc
#     src/main.cc
#     )

# add_executable (kuka_tests_hc
# ${RBDL_MODEL_TESTS_HDR_FILES} 
# ${KUKA_RBDL_MODEL_TESTS_HDR_FILES_HC} 
# ${KUKA_RBDL_MODEL_TESTS_SOURCE_FILES_HC})
# target_link_libraries (kuka_tests_hc
#     ${catkin_LIBRARIES} 
#     ${Eigen_LIBRARIES} 
#     ${AMBF_CLIENT_LIBRARY} 
#     ${RBDL_MODEL_LIBRARY} 
#     ${Boost_LIBRARIES} 
#     ${UNITTEST++_LIBRARY}
#     ${RBDL_LIBRARY}
#     pthread
#     )

# set_target_properties(kuka_tests_hc PROPERTIES
#     CXX_STANDARD 17
#     CXX_EXTENSIONS OFF
#     )
# ################END################

# ################START################
# #Executible kuka_tests

# # set (KUKA_RBDL_MODEL_TESTS_FILES
# #     include/${PROJECT_NAME}/AMBFParams.h
# #     include/${PROJECT_NAME}/KUKA.h
# #     )

# set(KUKA_RBDL_MODEL_TESTS_SOURCE_FILES
#     # src/ECMHCDynamicsTests.cc
#     # src/KukaHCKinematicsTests.cc
#     # src/KUKABodyTests.cc
#     # src/AMBFParams.cpp
#     # src/KUKA.cpp
#     # src/EigenUtilities.cpp
#     src/ModelFromParser/KUKA.cc
#     src/RBDLTestPrep.cpp
#     src/main.cc
#     )

# add_executable (kuka_tests
# ${RBDL_MODEL_TESTS_HDR_FILES} 
# # ${KUKA_RBDL_MODEL_TESTS_HDR_FILES} 
# ${KUKA_RBDL_MODEL_TESTS_SOURCE_FILES})
# target_link_libraries (kuka_tests
#     ${catkin_LIBRARIES} 
#     ${Eigen_LIBRARIES} 
#     ${AMBF_CLIENT_LIBRARY} 
#     ${APPLICATION_LIBRARY} 
#     ${RBDL_LIBRARY}
#     ${RBDL_MODEL_LIBRARY} 
#     ${YAML_CPP_LIBRARIES}
#     ${Boost_LIBRARIES} 
#     ${UNITTEST++_LIBRARY}
#     pthread
#     )

# set_target_properties(kuka_tests PROPERTIES
#     CXX_STANDARD 17
#     CXX_EXTENSIONS OFF
#     )
# ################END################

# ################START################
# #Executible Parallel Structure_hc_tests

# set (PS_RBDL_MODEL_TESTS_HDR_FILES
#     include/${PROJECT_NAME}/ParallelStructure.h
#     )

# set(PS_RBDL_MODEL_TESTS_SOURCE_FILES
#     src/PSHCKinematicsTests.cc
#     # src/PSBodyTests.cc
#     src/EigenUtilities.cpp
#     src/RBDLTestPrep.cc
#     src/main.cc
#     )

# add_executable (ps_tests_hc
# ${RBDL_MODEL_TESTS_HDR_FILES} 
# ${PS_RBDL_MODEL_TESTS_HDR_FILES} 
# ${PS_RBDL_MODEL_TESTS_SOURCE_FILES})
# target_link_libraries (ps_tests_hc
#     ${catkin_LIBRARIES} 
#     ${Eigen_LIBRARIES} 
#     ${AMBF_CLIENT_LIBRARY} 
#     ${RBDL_MODEL_LIBRARY} 
#     ${Boost_LIBRARIES} 
#     ${UNITTEST++_LIBRARY}
#     ${RBDL_LIBRARY}
#     pthread
#     )

# set_target_properties(ps_tests_hc PROPERTIES
#     CXX_STANDARD 17
#     CXX_EXTENSIONS OFF
#     )
# ################END################

################START################
#Executible ecm_hc_tests

set (ECM_RBDL_MODEL_TESTS_HDR_FILES
    include/${PROJECT_NAME}/AMBFParams.h
    include/${PROJECT_NAME}/ECM.h
    include/${PROJECT_NAME}/EigenUtilities.h
    )

set(ECM_RBDL_MODEL_TESTS_SOURCE_FILES
    # src/ECMHCDynamicsTests.cc
    src/ECMHCKinematicsTests.cc
    # src/ECMBodyTests.cc
    src/AMBFParams.cpp
    src/ECM.cpp
    src/EigenUtilities.cpp
    src/main.cc
    )

add_executable (ecm_tests_hc
${RBDL_MODEL_TESTS_HDR_FILES} 
${ECM_RBDL_MODEL_TESTS_HDR_FILES} 
${ECM_RBDL_MODEL_TESTS_SOURCE_FILES})
target_link_libraries (ecm_tests_hc
  ${catkin_LIBRARIES} 
  ${Eigen_LIBRARIES} 
  ${AMBF_CLIENT_LIBRARY} 
  # ${APPLICATION_LIBRARY} 
  ${RBDL_LIBRARY}
  ${RBDL_MODEL_LIBRARY} 
  # ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${UNITTEST++_LIBRARY}
  pthread
  )

set_target_properties(ecm_tests_hc PROPERTIES
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    )
################END################

# ################START################
# set(ECM_SOURCE_FILES
#     src/ECM_.cc
#     src/main.cc
#     )

# add_executable (ecm_
# ${ECM_SOURCE_FILES})
# target_link_libraries (ecm_
#     ${catkin_LIBRARIES}
#     ${RBDL_MODEL_LIBRARY} 
#     ${Boost_LIBRARIES} 
#     ${UNITTEST++_LIBRARY}
#     ${RBDL_LIBRARY}
#     pthread
#     )

# set_target_properties(ecm_ PROPERTIES
#     CXX_STANDARD 17
#     CXX_EXTENSIONS OFF
#     )
# ################END################

################START################
#Executible model from automation test cases

set (RBDL_MODEL_FROM_AUTOMATION_TESTS_HDR_FILES
    # include/${PROJECT_NAME}/AMBFParams.h
    include/${PROJECT_NAME}/ModelFromParser/RBDLModelFromAutomation.h
    include/${PROJECT_NAME}/ModelFromParser/TestCaseStruct.h
    )

set(RBDL_MODEL_FROM_AUTOMATION_TESTS_SOURCE_FILES
    src/ModelFromParser/RBDLModelTestCases.cc
    src/ModelFromParser/RBDLModelFromAutomation.cpp
    src/main.cc
    )

add_executable (rbdl_model_from_automation_tests
${RBDL_MODEL_TESTS_HDR_FILES} 
${RBDL_MODEL_FROM_AUTOMATION_TESTS_HDR_FILES} 
${RBDL_MODEL_FROM_AUTOMATION_TESTS_SOURCE_FILES})


target_link_libraries (rbdl_model_from_automation_tests
    ${catkin_LIBRARIES} 
    ${Eigen_LIBRARIES} 
    ${AMBF_CLIENT_LIBRARY} 
    ${APPLICATION_LIBRARY} 
    ${RBDL_LIBRARY}
    ${RBDL_MODEL_LIBRARY} 
    ${YAML_CPP_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${UNITTEST++_LIBRARY}
    pthread
    )

set_target_properties(rbdl_model_from_automation_tests PROPERTIES
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    )
################END################

################START################
# Test for validating World to body rotations to 
# create RBDL model without the need of AMBF 

set (RBDL_MODEL_ROTATIONS_TESTS_HDR_FILES
    # include/${PROJECT_NAME}/AMBFParams.h
    include/${PROJECT_NAME}/ModelFromParser/RBDLModelFromAutomation.h
    include/${PROJECT_NAME}/ModelFromParser/TestCaseStruct.h
    )

set(RBDL_MODEL_ROTATIONS_TESTS_SOURCE_FILES
    src/RotationTestCases.cc
    src/main.cc
    )

add_executable (rbdl_model_rotation_tests
${RBDL_MODEL_TESTS_HDR_FILES} 
# ${RBDL_MODEL_FROM_AUTOMATION_TESTS_HDR_FILES} 
${RBDL_MODEL_ROTATIONS_TESTS_SOURCE_FILES})


target_link_libraries (rbdl_model_rotation_tests
    ${catkin_LIBRARIES} 
    ${Eigen_LIBRARIES} 
    # ${AMBF_CLIENT_LIBRARY} 
    ${APPLICATION_LIBRARY} 
    ${RBDL_LIBRARY}
    ${RBDL_MODEL_LIBRARY} 
    # ${YAML_CPP_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${UNITTEST++_LIBRARY}
    pthread
    )

set_target_properties(rbdl_model_rotation_tests PROPERTIES
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    )
################END################
