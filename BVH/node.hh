#pragma once

#include <cstddef>
#include <limits>

#include "../common.hh"
#include "aabb.hh"
#include "common.hh"

namespace BVH {
ALIGNED_TYPE_(struct, 32) {
  AABB bounding_box;

  union {
    uint32_t first_primitive_index = 0;
    uint32_t left_child_index;
  };

  uint32_t number_of_primitives = 0;

  bool is_leaf() const { return number_of_primitives > 0; }
}

Node;
} // namespace BVH