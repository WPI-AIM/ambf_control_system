find_library(BOOST_LIBRARY
		NAMES libboost_system.so
		HINTS /usr/lib/x86_64-linux-gnu/
		NO_DEFAULT_PATH
		REQUIRED
	)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Dependent_Packages DEFAULT_MSG
  BOOST_LIBRARY
)

if(Dependent_Packages_FOUND)

  set(BOOST_LIBRARY_LIBRARY
    ${BOOST_LIBRARY}
  )

  set(BOOST_LIBRARY_LIBRARIES_PS
    ${BOOST_LIBRARY} PARENT_SCOPE
  )
	
endif()