#ifndef PADFLIE_BEHAVIORS_BASE__VISIBILITY_CONTROL_H_
#define PADFLIE_BEHAVIORS_BASE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PADFLIE_BEHAVIORS_BASE_EXPORT __attribute__ ((dllexport))
    #define PADFLIE_BEHAVIORS_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define PADFLIE_BEHAVIORS_BASE_EXPORT __declspec(dllexport)
    #define PADFLIE_BEHAVIORS_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef PADFLIE_BEHAVIORS_BASE_BUILDING_LIBRARY
    #define PADFLIE_BEHAVIORS_BASE_PUBLIC PADFLIE_BEHAVIORS_BASE_EXPORT
  #else
    #define PADFLIE_BEHAVIORS_BASE_PUBLIC PADFLIE_BEHAVIORS_BASE_IMPORT
  #endif
  #define PADFLIE_BEHAVIORS_BASE_PUBLIC_TYPE PADFLIE_BEHAVIORS_BASE_PUBLIC
  #define PADFLIE_BEHAVIORS_BASE_LOCAL
#else
  #define PADFLIE_BEHAVIORS_BASE_EXPORT __attribute__ ((visibility("default")))
  #define PADFLIE_BEHAVIORS_BASE_IMPORT
  #if __GNUC__ >= 4
    #define PADFLIE_BEHAVIORS_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define PADFLIE_BEHAVIORS_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PADFLIE_BEHAVIORS_BASE_PUBLIC
    #define PADFLIE_BEHAVIORS_BASE_LOCAL
  #endif
  #define PADFLIE_BEHAVIORS_BASE_PUBLIC_TYPE
#endif

#endif  // PADFLIE_BEHAVIORS_BASE__VISIBILITY_CONTROL_H_
