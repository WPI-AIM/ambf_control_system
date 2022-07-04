cmake_minimum_required(VERSION 3.20)
project(rbdl_model)

set(CMAKE_PREFIX_PATH _PATH "/opt/ros/melodic/;${CMAKE_MODULE_PATH}")

catkin_package( INCLUDE_DIRS include 
  LIBRARIES rbdl_model application
  CATKIN_DEPENDS ambf_msgs
#  DEPENDS system_lib
)

get_filename_component(PARENT_CMAKE ../cmake ABSOLUTE)
set (CMAKE_MODULE_PATH ${PARENT_CMAKE} )

LIST( APPEND CMAKE_MODULE_PATH ${PARENT_DIR} )
find_package(RBDL REQUIRED)

FIND_PACKAGE (yaml-cpp REQUIRED)
FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
include_directories(include 
${catkin_INCLUDE_DIRS} 
  ${RBDL_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR} 
  ${RBDL_INCLUDE_DIR}
  )


# set (RBDL_MODEL_INCLUDE_DIRS include)
# set (RBDL_MODEL_LIBRARIES rbdl_model)

#add_executable(${EXECUTABLE_NAME})

set (HDR_FILES
  include/${PROJECT_NAME}/BodyParam.h
  include/${PROJECT_NAME}/JointParam.h
  include/${PROJECT_NAME}/ParseADF.h
  include/${PROJECT_NAME}/BuildRBDLModel.h
  # include/${PROJECT_NAME}/AMBFParams.h
  include/${PROJECT_NAME}/ModelGraph.h
  include/${PROJECT_NAME}/MinHeap.h
  include/${PROJECT_NAME}/MinHeapStructs.h
  include/${PROJECT_NAME}/ModelStructs.h
  include/${PROJECT_NAME}/GraphEdge.h
  include/${PROJECT_NAME}/RBDLUtilities.h
  include/${PROJECT_NAME}/RBDLModelErrors.h
)


set(SOURCE_FILES
  src/BodyParam.cpp
  src/JointParam.cpp
  src/ParseADF.cpp
  src/BuildRBDLModel.cpp
  # src/AMBFParams.cpp
  src/ModelGraph.cpp
  src/MinHeap.cpp
  src/RBDLUtilities.cpp
  src/RBDLModelErrors.cpp
  )

## Declare a C++ library
add_library(rbdl_model ${HDR_FILES} ${SOURCE_FILES})

add_dependencies(rbdl_model ${catkin_EXPORTED_TARGETS})


#add_executable (${PROJECT_NAME} ${HDR_FILES} ${SOURCE_FILES})
## Specify libraries to link a library or executable target against
target_link_libraries(rbdl_model PUBLIC
  # ambf_client_lib
  ${catkin_LIBRARIES} 
  ${Eigen_LIBRARIES} 
  ${RBDL_MODEL_LIBRARY} 
  ${Boost_LIBRARIES} 
  ${RBDL_LIBRARY}
  ${YAML_CPP_LIBRARIES}
  ${APPLICATION_LIBRARY}  
  pthread
  )

# target_link_libraries(rbdl_model PRIVATE )

set_target_properties(rbdl_model PROPERTIES
 CXX_STANDARD 17
 CXX_STANDARD_REQUIRED YES
 )

set (RBDL_MODEL_INCLUDE_DIRS include)
set (RBDL_MODEL_LIBRARIES rbdl_model)