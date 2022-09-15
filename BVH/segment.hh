#pragma once

#include <array>

#include "../vec3.hh"

namespace BVH {
struct Vec2 {
  float x, y;
};

using Segment2D = std::array<Vec2, 2>;
using Segment3D = std::array<Vec3, 2>;
} // namespace BVH