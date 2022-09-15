#pragma once

#include "../vec3.hh"
#include "barycentric.hh"
#include "segment.hh"
#include "triangle.hh"

struct Segment_Triangle_Intersection_Result {
  std::array<Vec3, 2> points;
  size_t num_points = 0;
};

int signof(float value, float epsilon) {
  if (value > epsilon) {
    return 1;
  }
  if (value < epsilon) {
    return -1;
  }
  return 0;
}

static bool do_segment_intersect_triangle(const BVH::Segment3D &segment,
                                          const BVH::Triangle &triangle) {
  Vec3 s1 = segment[0] - triangle.verts[0];
  Vec3 s2 = segment[1] - triangle.verts[0];
  Vec3 triangle_normal = triangle.calc_normal();

  float d1 = triangle_normal.dot(s1);
  float d2 = triangle_normal.dot(s2);

  int sign1 = signof(d1, 0.0001f);
  int sign2 = signof(d2, 0.0001f);

  if ((sign1 == sign2) && (sign1 != 0)) {
    // Both segment points are on the same side of the triangle supporting
    // plane, but not coplanar, no intersection is possible
    return false;
  }

  Barycentric_Info barycentric_info(triangle);

  if ((sign1 == 0) && (sign2 == 0)) {
    // Coplanar case
    /* TODO:
    1. if any point of the two segment points are inside triangle, include
    it in the intersection result
    2. if both points are inside triangle return that result
    3. if not, intersect segment with all triangle segments
    4. note there might be no intersections, if the segment doesn't
    intersect any of triangle segments, nor does it have any point inside
    triangle
    */

    if (barycentric_info.is_inside_triangle(segment[0]) &&
        barycentric_info.is_inside_triangle(segment[1])) {
      return true;
    }

    return false;
  }

  // Non-coplanar case
  // Intersect segment supporting ray with triangle supporting plane
  // Reference: https://stackoverflow.com/a/23976134/8094047
  Vec3 ray_direction = (segment[1] - segment[0]).normalized();
  float denom = triangle_normal.dot(ray_direction);
  float t = (triangle.verts[0] - segment[0]).dot(triangle_normal) / denom;
  Vec3 intersection_point = t * ray_direction + segment[0];

  return barycentric_info.is_inside_triangle(intersection_point);
}

static Segment_Triangle_Intersection_Result
intersect_segment_triangle(const BVH::Segment3D &segment,
                           const BVH::Triangle &triangle) {
  Vec3 s1 = segment[0] - triangle.verts[0];
  Vec3 s2 = segment[1] - triangle.verts[0];
  Vec3 triangle_normal = triangle.calc_normal();

  float d1 = triangle_normal.dot(s1);
  float d2 = triangle_normal.dot(s2);

  int sign1 = signof(d1, 0.0001f);
  int sign2 = signof(d2, 0.0001f);

  if ((sign1 == sign2) && (sign1 != 0)) {
    // Both segment points are on the same side of the triangle supporting
    // plane, no intersection is possible
    return {{}, 0};
  }

  Barycentric_Info barycentric_info(triangle);

  if ((sign1 == 0) && (sign2 == 0)) {
    Segment_Triangle_Intersection_Result res;
    if (barycentric_info.is_inside_triangle(segment[0])) {
      res.points[res.num_points++] = segment[0];
    }
    if (barycentric_info.is_inside_triangle(segment[1])) {
      res.points[res.num_points++] = segment[1];
    }
    if (res.num_points == 2) {
      return res;
    }
    // Coplanar case
    /* TODO:
    1. if any point of the two segment points are inside triangle, include
    it in the intersection result
    2. if both points are inside triangle return that result
    3. if not, intersect segment with all triangle segments
    4. note there might be no intersections, if the segment doesn't
    intersect any of triangle segments, nor does it have any point inside
    triangle
    */
    return {{}, 0};
  }

  // Non-coplanar case
  // Intersect segment supporting ray with triangle supporting plane
  // Reference: https://stackoverflow.com/a/23976134/8094047
  Vec3 ray_direction = (segment[1] - segment[0]).normalized();
  float denom = triangle_normal.dot(ray_direction);
  float t = (triangle.verts[0] - segment[0]).dot(triangle_normal) / denom;
  Vec3 intersection_point = t * ray_direction + segment[0];

  if (barycentric_info.is_inside_triangle(intersection_point)) {
    return {{intersection_point, {}}, 1};
  }

  return {{}, 0};
}
