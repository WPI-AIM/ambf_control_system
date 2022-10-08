find_path(ROS_INCLUDE_DIR
		NAMES ros/ros.h
		HINTS /opt/ros/noetic/include
  )

find_path(AMBF_CLIENT_INCLUDE_DIR
		NAMES ambf_client/ambf_client.h
		HINTS /mnt/OneTB/localcodebase/ambf_repos/ambf/ambf_ros_modules/ambf_client/include
  )

find_path(AMBF_MSGS_INCLUDE_DIR
	NAMES ambf_msgs/ActuatorState.h
	HINTS /mnt/OneTB/localcodebase/ambf_repos/ambf/build/devel/include
)




find_library(AMBF_CLIENT_LIBRARY
	NAMES libambf_client_cpp.so
	HINTS /mnt/OneTB/localcodebase/ambf_repos/ambf/build/devel/lib
)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(AMBF_CLIENT DEFAULT_MSG
	ROS_INCLUDE_DIR 
	AMBF_CLIENT_INCLUDE_DIR 
	AMBF_CLIENT_LIBRARY
	AMBF_MSGS_INCLUDE_DIR)


if(AMBF_CLIENT_FOUND)

	set(AMBF_CLIENT_INCLUDE_DIRS
	${ROS_INCLUDE_DIR}
	${AMBF_CLIENT_INCLUDE_DIR}
	${AMBF_MSGS_INCLUDE_DIR})
	
	set(AMBF_CLIENT_LIBRARIES
	${AMBF_CLIENT_LIBRARY})

endif()