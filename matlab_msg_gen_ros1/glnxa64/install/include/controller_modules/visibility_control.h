#ifndef CONTROLLER_MODULES__VISIBILITY_CONTROL_H_
#define CONTROLLER_MODULES__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONTROLLER_MODULES_EXPORT __attribute__ ((dllexport))
    #define CONTROLLER_MODULES_IMPORT __attribute__ ((dllimport))
  #else
    #define CONTROLLER_MODULES_EXPORT __declspec(dllexport)
    #define CONTROLLER_MODULES_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONTROLLER_MODULES_BUILDING_LIBRARY
    #define CONTROLLER_MODULES_PUBLIC CONTROLLER_MODULES_EXPORT
  #else
    #define CONTROLLER_MODULES_PUBLIC CONTROLLER_MODULES_IMPORT
  #endif
  #define CONTROLLER_MODULES_PUBLIC_TYPE CONTROLLER_MODULES_PUBLIC
  #define CONTROLLER_MODULES_LOCAL
#else
  #define CONTROLLER_MODULES_EXPORT __attribute__ ((visibility("default")))
  #define CONTROLLER_MODULES_IMPORT
  #if __GNUC__ >= 4
    #define CONTROLLER_MODULES_PUBLIC __attribute__ ((visibility("default")))
    #define CONTROLLER_MODULES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONTROLLER_MODULES_PUBLIC
    #define CONTROLLER_MODULES_LOCAL
  #endif
  #define CONTROLLER_MODULES_PUBLIC_TYPE
#endif
#endif  // CONTROLLER_MODULES__VISIBILITY_CONTROL_H_
// Generated 09-Jun-2021 13:58:38
// Copyright 2019-2020 The MathWorks, Inc.
