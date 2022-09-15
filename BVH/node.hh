#pragma once

#include <cstddef>
#include <limits>

#include "aabb.hh"
#include "common.hh"

namespace BVH {
struct Node {
  AABB bounding_box;
  size_t left_child_index, right_child_index;
  size_t first_primitive_index, last_primitive_index;

  Node() {
    left_child_index = right_child_index = INVALID_INDEX;
    first_primitive_index = last_primitive_index = INVALID_INDEX;
  }

  bool is_leaf() const {
    return left_child_index == INVALID_INDEX &&
           right_child_index == INVALID_INDEX;
  }
};
} // namespace BVH