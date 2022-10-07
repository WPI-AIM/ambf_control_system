set(RBDL_DIR
  out/build/external/rbdl
  )

find_path(RBDL_INCLUDE_DIR
		NAMES rbdl/Body.h
		HINTS external/rbdl/include
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RBDL DEFAULT_MSG
  RBDL_INCLUDE_DIR
  )


if(RBDL_FOUND)
  
  set(RBDL_INCLUDE_DIRS  
    ${RBDL_INCLUDE_DIR})

endif()
