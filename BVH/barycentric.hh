#pragma once

#include "../vec3.hh"
#include "triangle.hh"

struct Barycentric_Info {
  // Reference: https://blackpawn.com/texts/pointinpoly/
  Vec3 triangle_v0;
  Vec3 v0, v1;
  float dot00, dot01, dot11, inv_denom;

  explicit Barycentric_Info(const BVH::Triangle &triangle) {
    triangle_v0 = triangle.verts[0];
    v0 = triangle.verts[2] - triangle.verts[0];
    v1 = triangle.verts[1] - triangle.verts[0];
    dot00 = dot(v0, v0);
    dot01 = dot(v0, v1);
    dot11 = dot(v1, v1);
    inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);
  }

  bool is_inside_triangle(const Vec3 &point) const {
    Vec3 v2 = point - triangle_v0;
    float dot02 = dot(v0, v2);
    float dot12 = dot(v1, v2);
    float u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    float v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
    return (u >= 0) && (v >= 0) && (u + v < 1);
  }
};