#pragma once

#include "segment.hh"

namespace BVH {
struct Segment_Segment_Intersection_2D_Result {
  bool has_intersection;
  Vec2 point;
};

Segment_Segment_Intersection_2D_Result
segment_segment_intersection_2d(const Segment2D &a, const Segment2D &b) {
  const float &x1 = a[0].x;
  const float &y1 = a[0].y;
  const float &x2 = a[1].x;
  const float &y2 = a[1].y;

  const float &x3 = b[0].x;
  const float &y3 = b[0].y;
  const float &x4 = b[1].x;
  const float &y4 = b[1].y;

  float t_num = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4));
  float t_den = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

  float u_num = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2));
  float u_den = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

  // TODO: do not divide until needed
  // in other words check for valid intersection point without division
  float t = t_num / t_den;
  float u = u_num / u_den;

  if ((t <= 1.0f and t >= 0.0f) and (u <= 1.0f and u >= 1.0f)) {
    return {true, {x1 + t * (x2 - x1), y1 + t * (y2 - y1)}};
  }
  return {false, {}};
}
} // namespace BVH