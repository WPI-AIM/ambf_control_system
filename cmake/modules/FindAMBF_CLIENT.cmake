find_path(ROS_INCLUDE_DIR
    NAMES ros/ros.h
    HINTS /opt/ros/noetic/include
  )


find_path(AMBF_CLIENT_INCLUDE_DIR
		NAMES ambf_client/ambf_client.h
		HINTS ${CMAKE_SOURCE_DIR}/../ambf/ambf_ros_modules/ambf_client
    PATH_SUFFIXES include
  )

find_path(AMBF_MSGS_INCLUDE_DIR
		NAMES ambf_msgs/ActuatorState.h
		HINTS ${CMAKE_SOURCE_DIR}/../ambf/build/devel
    PATH_SUFFIXES include
  )

find_library(AMBF_CLIENT_LIBRARY
		NAMES libambf_client_cpp.so
		HINTS ${CMAKE_SOURCE_DIR}/../ambf/build/devel/lib
		NO_DEFAULT_PATH
		REQUIRED
	)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(AMBF_CLIENT DEFAULT_MSG
	ROS_INCLUDE_DIR
	AMBF_CLIENT_INCLUDE_DIR
	AMBF_MSGS_INCLUDE_DIR
	AMBF_CLIENT_LIBRARY
)


if(AMBF_CLIENT_FOUND)

  set(ROS_INCLUDE_DIRS
    ${ROS_INCLUDE_DIR}
  )

  set(AMBF_CLIENT_INCLUDE_DIRS
    ${AMBF_CLIENT_INCLUDE_DIR}
  )

  set(AMBF_MSGS_INCLUDE_DIRS
    ${AMBF_MSGS_INCLUDE_DIR}
  )

  set(AMBF_CLIENT_LIBRARIES
    ${AMBF_CLIENT_LIBRARY}
  )


  set(ROS_INCLUDE_DIRS_PS 
    ${ROS_INCLUDE_DIR} PARENT_SCOPE
  )

  set(AMBF_CLIENT_INCLUDE_DIRS_PS
    ${AMBF_CLIENT_INCLUDE_DIR} PARENT_SCOPE
  )

  set(AMBF_MSGS_INCLUDE_DIRS_PS
    ${AMBF_MSGS_INCLUDE_DIR} PARENT_SCOPE
  )

  set(AMBF_CLIENT_LIBRARIES_PS
    ${AMBF_CLIENT_LIBRARY} PARENT_SCOPE
  )
	
endif()