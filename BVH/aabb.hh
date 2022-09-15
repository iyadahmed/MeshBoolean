#pragma once

#include "../vec3.hh"

namespace BVH {
struct AABB {
  Vec3 max, min;
};
} // namespace BVH