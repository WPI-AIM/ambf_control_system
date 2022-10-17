find_path(spdlog_INCLUDE_DIR
		NAMES spdlog/spdlog.h
    HINTS ${spdlog_SOURCE_DIR}
    PATH_SUFFIXES include
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(spdlog DEFAULT_MSG
  spdlog_INCLUDE_DIR)


if(spdlog_FOUND)
  set(spdlog_INCLUDE_DIRS  
  ${spdlog_INCLUDE_DIR})

  set(spdlog_INCLUDE_DIRS_PS  
  ${spdlog_INCLUDE_DIR} PARENT_SCOPE)
endif()