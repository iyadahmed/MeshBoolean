#pragma once

#include <limits>
#include <stack>
#include <stdexcept>

#include "bvh.hh"

namespace BVH {
size_t closest_triangle(const BVH &bvh, const Vec3 &point,
                        float max_distance = 0.001f);
}
