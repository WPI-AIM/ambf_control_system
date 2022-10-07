find_path(CATCH2_INCLUDE_DIR
		NAMES catch2/catch.hpp
		HINTS external/Catch2/single_include
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CATCH2 DEFAULT_MSG
  CATCH2_INCLUDE_DIR)


if(CATCH2_FOUND)
  set(CATCH2_INCLUDE_DIRS  
  ${CATCH2_INCLUDE_DIR})
endif()