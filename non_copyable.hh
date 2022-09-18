#pragma once

// From Blender
/**
 * A type that inherits from NonCopyable cannot be copied anymore.
 */
class NonCopyable {
public:
  /* Disable copy construction and assignment. */
  NonCopyable(const NonCopyable &other) = delete;
  NonCopyable &operator=(const NonCopyable &other) = delete;

  /* Explicitly enable default construction, move construction and move
   * assignment. */
  NonCopyable() = default;
  NonCopyable(NonCopyable &&other) = default;
  NonCopyable &operator=(NonCopyable &&other) = default;
};