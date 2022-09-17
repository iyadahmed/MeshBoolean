#pragma once

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
#define likely(expr) (expr)
#define unlikely(expr) (expr)
#else
#define likely(expr) __builtin_expect((bool)(expr), true)
#define unlikely(expr) __builtin_expect((bool)(expr), false)
#endif

#include <type_traits>

template <typename T, typename = typename std::enable_if<
                          std::is_arithmetic<T>::value, T>::type>
bool is_in_range_inclusive(const T &value, T range_start, T range_end) {
  return value >= range_start and value <= range_end;
}

// https://stackoverflow.com/a/12654801/8094047
#if defined(_MSC_VER)
#define ALIGNED_(x) __declspec(align(x))
#else
#if defined(__GNUC__)
#define ALIGNED_(x) __attribute__((aligned(x)))
#endif
#endif

#define ALIGNED_TYPE_(t, x) typedef t ALIGNED_(x)
