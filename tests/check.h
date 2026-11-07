#pragma once

#include <cstdio>
#include <cstdlib>

// Unlike assert, checks and their expressions still run with NDEBUG defined.
#define CHECK(expression) \
  do { \
    if (!(expression)) { \
      std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #expression); \
      std::abort(); \
    } \
  } while (false)
