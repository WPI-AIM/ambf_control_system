SET (AMBF_CLIENT_FOUND FALSE)


UNSET( AMBF_CLIENT_INCLUDE_DIR              CACHE)   
UNSET( AMBF_CLIENT_LIBRARY                  CACHE)  

FIND_PATH (AMBF_CLIENT_INCLUDE_DIR ambf_client/ambf_client.h
HINTS
/localcodebase/ambfnags92/ambf/ambf_controller/ambf_client/include
/usr/local/include
/usr/include
)

FIND_LIBRARY (AMBF_CLIENT_LIBRARY ambf_client
PATHS
/localcodebase/ambfnags92/ambf/build/devel/lib
)

IF (AMBF_CLIENT_INCLUDE_DIR AND AMBF_CLIENT_LIBRARY)
  SET (AMBF_CLIENT_FOUND TRUE)
ELSE(AMBF_CLIENT_INCLUDE_DIR AND AMBF_CLIENT_LIBRARY)   
  SET (AMBF_CLIENT_FOUND FALSE) 
  MESSAGE (SEND_ERROR " Could not find AMBF_CLIENT.")
ENDIF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)

IF (AMBF_CLIENT_FOUND)
    MESSAGE(STATUS "Found AMBF: ${AMBF_CLIENT_LIBRARY}")
ELSE(AMBF_CLIENT_FOUND)
    MESSAGE(STATUS " NOt Found AMBF")
ENDIF (RBDL_FOUND)