find_path(Catch2_INCLUDE_DIR
		NAMES catch2/catch.hpp
    HINTS ${Catch2_SOURCE_DIR}
    PATH_SUFFIXES single_include
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Catch2 DEFAULT_MSG
  Catch2_INCLUDE_DIR)


if(Catch2_FOUND)
  set(Catch2_INCLUDE_DIRS  
  ${Catch2_INCLUDE_DIR})

  set(Catch2_INCLUDE_DIRS_PS  
  ${Catch2_INCLUDE_DIR} PARENT_SCOPE)
endif()