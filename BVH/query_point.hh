#pragma once

#include <limits>
#include <stack>
#include <stdexcept>

#include "bvh.hh"

namespace BVH {
bool contains_point(const BVH &bvh, const Vec3 &point, size_t node_index = 0);
}
