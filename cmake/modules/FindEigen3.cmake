find_path(Eigen3_INCLUDE_DIR
		NAMES Eigen/Dense
    HINTS ${Eigen3_SOURCE_DIR}
    PATH_SUFFIXES eigen3
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3 DEFAULT_MSG
  Eigen3_INCLUDE_DIR)


if(Eigen3_FOUND)
  set(Eigen3_INCLUDE_DIRS  
  ${Eigen3_INCLUDE_DIR})

  set(Eigen3_INCLUDE_DIRS_PS
  ${Eigen3_INCLUDE_DIR} PARENT_SCOPE)
endif()