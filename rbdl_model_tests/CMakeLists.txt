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



set (ambf_msgs_DIR ${CMAKE_MODULE_PATH})

set (AMBF_CLIENT_LIBRARY "/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf/build/devel/lib/libambf_client_cpp.so")

FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
#find_package(GTest REQUIRED)

include_directories("/mnt/OneTB/localcodebase/ambf_repos/ambfvtwodotzero/ambf/ambf_ros_modules/ambf_client/include/")
FIND_PACKAGE (ambf_client REQUIRED)
include_directories(${AMBF_CLIENT_INCLUDE_DIR})


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


# Look for unittest++
FIND_PACKAGE (UnitTest++ REQUIRED)
INCLUDE_DIRECTORIES (${UNITTEST++_INCLUDE_DIR})

include_directories( include ${AMBF_CLIENT_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${RBDL_INCLUDE_DIR})


#find_package (ambf_client REQUIRED)
find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories (${GLFW_INCLUDE_DIRS}
                     ${PROJECT_NAME}/include
)


# set (HDR_FILES
#     include/${PROJECT_NAME}/rbdl_model_test.h)

# set(SOURCE_FILES
#     src/rbdl_model_test.cpp
#     #src/main.cpp
#     )


# set (RBDL_MODEL_TESTS_HDR_FILES
# #    include/${PROJECT_NAME}/RBDLBodyTests.h
#     include/${PROJECT_NAME}/RBDLTestPrep.h
#     )

# set(RBDL_MODEL_TESTS_SOURCE_FILES
#     src/RBDLBodyTests.cpp
#     src/RBDLJointTests.cpp
#     src/RBDLTestPrep.cpp
#     src/main.cpp
#     )

find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${GLFW_INCLUDE_DIRS})

set (KUKA_RBDL_MODEL_TESTS_HDR_FILES
    include/${PROJECT_NAME}/Kuka.h
    include/${PROJECT_NAME}/EigenUtilities.h
    include/${PROJECT_NAME}/rbdl_tests.h
    include/${PROJECT_NAME}/RBDLTestPrep.h    
    )

# ################START################
# # Executible kuka_tests
# set(KUKA_RBDL_MODEL_TESTS_SOURCE_FILES
#     src/KukaKinematicsTests.cc
#     #src/KukaBodyTests.cc
#     src/EigenUtilities.cpp
#     src/RBDLTestPrep.cc
#     src/main.cc
#     )

# add_executable (kuka_tests 
      #${KUKA_RBDL_MODEL_TESTS_HDR_FILES} 
      #${KUKA_RBDL_MODEL_TESTS_SOURCE_FILES})
# target_link_libraries (kuka_tests 
#     ${AMBF_CLIENT_LIBRARY} 
#     ${RBDL_MODEL_LIBRARY}
#     ${GLFW_LIBRARIES} 
#     ${Boost_LIBRARIES} 
#     ${Eigen_LIBRARIES} 
#     ${UNITTEST++_LIBRARY}
#     ${RBDL_LIBRARY}
#     pthread
#     )
################END################


################START################
#Executible kuka_hc_tests
set(KUKA_RBDL_MODEL_TESTS_SOURCE_FILES
    #src/KukaHCDynamicsTests.cc
    src/KukaHCKinematicsTests.cc
    src/EigenUtilities.cpp
    src/RBDLTestPrep.cc
    src/main.cc
    )

add_executable (kuka_tests_hc
${KUKA_RBDL_MODEL_TESTS_HDR_FILES} 
${KUKA_RBDL_MODEL_TESTS_SOURCE_FILES})
target_link_libraries (kuka_tests_hc
    ${AMBF_CLIENT_LIBRARY} 
    ${RBDL_MODEL_LIBRARY}
    ${GLFW_LIBRARIES} 
    ${Boost_LIBRARIES} 
    # ${Eigen_LIBRARIES} 
    ${UNITTEST++_LIBRARY}
    ${RBDL_LIBRARY}
    pthread
    )
################END################

################START################
set (ECM_RBDL_MODEL_TESTS_HDR_FILES
    include/${PROJECT_NAME}/ECM.h
    include/${PROJECT_NAME}/EigenUtilities.h
    include/${PROJECT_NAME}/rbdl_tests.h
    include/${PROJECT_NAME}/RBDLTestPrep.h    
    )

# Executible ECM_tests
set(ECM_HC_RBDL_MODEL_TESTS_SOURCE_FILES
    src/ECMHCKinematicsTests.cc
    # src/ECMBodyTests.cc
    src/EigenUtilities.cpp
    src/RBDLTestPrep.cc
    src/main.cc
    )

add_executable (ecm_tests_hc 
      ${ECM_RBDL_MODEL_TESTS_HDR_FILES} 
      ${ECM_HC_RBDL_MODEL_TESTS_SOURCE_FILES})
target_link_libraries (ecm_tests_hc
    ${AMBF_CLIENT_LIBRARY} 
    ${RBDL_MODEL_LIBRARY}
    ${GLFW_LIBRARIES} 
    ${Boost_LIBRARIES} 
    # ${Eigen_LIBRARIES} 
    ${UNITTEST++_LIBRARY}
    ${RBDL_LIBRARY}
    pthread
    )
##############END################

################START################
set (PS_RBDL_MODEL_TESTS_HDR_FILES
    include/${PROJECT_NAME}/ParallelStructure.h
    include/${PROJECT_NAME}/EigenUtilities.h
    include/${PROJECT_NAME}/rbdl_tests.h
    include/${PROJECT_NAME}/RBDLTestPrep.h    
    )

# Executible Parallel_Structure_tests
set(PS_HC_RBDL_MODEL_TESTS_SOURCE_FILES
    src/ParallelStructureHCKinematicsTests.cc
    # src/ParallelStructureBodyTest.cc
    src/EigenUtilities.cpp
    src/RBDLTestPrep.cc
    src/main.cc
    )

add_executable (ps_tests_hc 
      ${PS_RBDL_MODEL_TESTS_HDR_FILES} 
      ${PS_HC_RBDL_MODEL_TESTS_SOURCE_FILES})
target_link_libraries (ps_tests_hc
    ${AMBF_CLIENT_LIBRARY} 
    ${RBDL_MODEL_LIBRARY}
    ${GLFW_LIBRARIES} 
    ${Boost_LIBRARIES} 
    # ${Eigen_LIBRARIES} 
    ${UNITTEST++_LIBRARY}
    ${RBDL_LIBRARY}
    pthread
    )
##############END################