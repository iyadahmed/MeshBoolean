#pragma once

#include <limits>

#include "../vec3.hh"
#include "aabb.hh"
#include "segment.hh"

static bool do_segment_intersect_aabb(const BVH::Segment3D &segment,
                                      const BVH::AABB &aabb) {
  // Reference:
  // https://gamedev.net/forums/topic/338987-aabb-line-segment-intersection-test/3209917/

  const Vec3 &p1 = segment[0];
  const Vec3 &p2 = segment[1];

  constexpr float EPSILON = std::numeric_limits<float>::epsilon() * 100;
  Vec3 d = (p2 - p1) * 0.5f;
  Vec3 e = (aabb.max - aabb.min) * 0.5f;
  Vec3 c = p1 + d - (aabb.min + aabb.max) * 0.5f;
  Vec3 ad = d.absolute();
  if (std::abs(c[0]) > e[0] + ad[0])
    return false;
  if (std::abs(c[1]) > e[1] + ad[1])
    return false;
  if (std::abs(c[2]) > e[2] + ad[2])
    return false;
  if (std::abs(d[1] * c[2] - d[2] * c[1]) >
      e[1] * ad[2] + e[2] * ad[1] + EPSILON)
    return false;
  if (std::abs(d[2] * c[0] - d[0] * c[2]) >
      e[2] * ad[0] + e[0] * ad[2] + EPSILON)
    return false;
  if (std::abs(d[0] * c[1] - d[1] * c[0]) >
      e[0] * ad[1] + e[1] * ad[0] + EPSILON)
    return false;
  return true;
}