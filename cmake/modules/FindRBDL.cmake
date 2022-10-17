find_path(RBDL_INCLUDE_DIR
		NAMES rbdl/Body.h
		HINTS ${RBDL_SOURCE_DIR}
    PATH_SUFFIXES include
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RBDL DEFAULT_MSG
  RBDL_INCLUDE_DIR)


if(RBDL_FOUND)
  set(RBDL_INCLUDE_DIRS  
  ${RBDL_INCLUDE_DIR})

  set(RBDL_INCLUDE_DIRS_PS
  ${RBDL_INCLUDE_DIR} PARENT_SCOPE)
endif()