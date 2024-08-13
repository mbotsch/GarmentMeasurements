#pragma once

#if defined(_MSC_VER)
#define FBX_WARNINGS_DISABLE_BEGIN //TODO
#define FBX_WARNINGS_DISABLE_END //TODO
#elif defined(__GNUC__) || defined(__clang__)
#define FBX_WARNINGS_DISABLE_BEGIN \
  _Pragma("GCC diagnostic push") \
  _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"") \
  _Pragma("GCC diagnostic ignored \"-Wdeprecated-copy\"") \
  _Pragma("GCC diagnostic ignored \"-Wclass-memaccess\"")
  _Pragma("GCC diagnostic ignored \"-Wpedantic\"")
#define FBX_WARNINGS_DISABLE_END \
  _Pragma("GCC diagnostic pop")
#else
#define FBX_WARNINGS_DISABLE_BEGIN
#define FBX_WARNINGS_DISABLE_END
#endif

#if 0
// TODO(swenninger): find warnings for MSVC and apple
  __pragma(warning(push, 0)) \
  __pragma(warning(disable: 4701)) \
  __pragma(warning(disable: 4702))
  __pragma(warning(default: 4701)) \
  __pragma(warning(default: 4702)) \
  __pragma(warning(pop ))
#endif
