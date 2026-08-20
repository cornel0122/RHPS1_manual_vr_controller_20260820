#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define ANAAvatarController_DLLIMPORT __declspec(dllimport)
#  define ANAAvatarController_DLLEXPORT __declspec(dllexport)
#  define ANAAvatarController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define ANAAvatarController_DLLIMPORT __attribute__((visibility("default")))
#    define ANAAvatarController_DLLEXPORT __attribute__((visibility("default")))
#    define ANAAvatarController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define ANAAvatarController_DLLIMPORT
#    define ANAAvatarController_DLLEXPORT
#    define ANAAvatarController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef ANAAvatarController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define ANAAvatarController_DLLAPI
#  define ANAAvatarController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef ANAAvatarController_EXPORTS
#    define ANAAvatarController_DLLAPI ANAAvatarController_DLLEXPORT
#  else
#    define ANAAvatarController_DLLAPI ANAAvatarController_DLLIMPORT
#  endif // ANAAvatarController_EXPORTS
#  define ANAAvatarController_LOCAL ANAAvatarController_DLLLOCAL
#endif // ANAAvatarController_STATIC
