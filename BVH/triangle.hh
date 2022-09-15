#pragma once

#include <cassert>
#include <cmath> // for std::isfinite

#include "../vec3.hh"
#include "aabb.hh"

namespace BVH {
class Triangle {
public:
  std::array<Vec3, 3> verts;
  Vec3 cached_centroid;
  AABB cached_bounding_box;

  Vec3 calc_centroid() const {
    return {
        (verts[0].x + verts[1].x + verts[2].x) / 3,
        (verts[0].y + verts[1].y + verts[2].y) / 3,
        (verts[0].z + verts[1].z + verts[2].z) / 3,
    };
  }

  Vec3 calc_normal() const {
    return (verts[1] - verts[0]).cross(verts[2] - verts[0]).normalized();
  }

  AABB calc_bounding_box() const {
#ifndef NDEBUG
    for (const auto &v : verts) {
      assert(std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z));
    }
#endif
    AABB out;
    out.min = Vec3::min(verts[0], Vec3::min(verts[1], verts[2]));
    out.max = Vec3::max(verts[0], Vec3::max(verts[1], verts[2]));
    return out;
  }
};
} // namespace BVH