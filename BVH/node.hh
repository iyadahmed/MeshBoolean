#pragma once

#include <cstddef>
#include <limits>

#include "aabb.hh"
#include "common.hh"

namespace BVH {
struct Node {
  AABB bounding_box;

  union {
    uint32_t first_primitive_index = 0;
    uint32_t left_child_index;
  };

  uint32_t number_of_primitives = 0;

  bool is_leaf() const { return number_of_primitives > 0; }
} __attribute__((aligned(32)));
} // namespace BVH