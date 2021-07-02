#ifndef RBDL_SERVER__VISIBILITY_CONTROL_H_
#define RBDL_SERVER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RBDL_SERVER_EXPORT __attribute__ ((dllexport))
    #define RBDL_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define RBDL_SERVER_EXPORT __declspec(dllexport)
    #define RBDL_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef RBDL_SERVER_BUILDING_LIBRARY
    #define RBDL_SERVER_PUBLIC RBDL_SERVER_EXPORT
  #else
    #define RBDL_SERVER_PUBLIC RBDL_SERVER_IMPORT
  #endif
  #define RBDL_SERVER_PUBLIC_TYPE RBDL_SERVER_PUBLIC
  #define RBDL_SERVER_LOCAL
#else
  #define RBDL_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define RBDL_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define RBDL_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define RBDL_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RBDL_SERVER_PUBLIC
    #define RBDL_SERVER_LOCAL
  #endif
  #define RBDL_SERVER_PUBLIC_TYPE
#endif
#endif  // RBDL_SERVER__VISIBILITY_CONTROL_H_
// Generated 09-Jun-2021 13:58:42
// Copyright 2019-2020 The MathWorks, Inc.
