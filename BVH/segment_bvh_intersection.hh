#pragma once

#include <cstddef>
#include <vector>

#include "bvh.hh"
#include "segment.hh"
#include "segment_aabb_intersection.hh"
#include "segment_triangle_intersection.hh"
#include "triangle.hh"

struct Intersection_Point {
  size_t triangle_index;
  float c1, c2;
};

void intersect(const BVH::BVH &bvh, const BVH::Segment3D &segment,
               std::vector<Intersection_Point> &output, size_t node_index = 0) {
  const BVH::Node &node = bvh.nodes[node_index];
  if (!do_segment_intersect_aabb(segment, node.bounding_box)) {
    return;
  }
  if (node.is_leaf()) {
    for (size_t i = node.first_primitive_index; i <= node.last_primitive_index;
         i++) {
      const BVH::Triangle &t = bvh.triangles[i];
      auto intersection_result = intersect_segment_triangle(segment, t);
      for (size_t pi = 0; pi < intersection_result.num_points; pi++) {
        const Vec3 &v = intersection_result.points[pi];
        Vec3 b1 = t.verts[1] - t.verts[0];
        Vec3 b2 = t.verts[2] - t.verts[0];
        Vec3 rel = v - t.verts[0];
        float c1 = rel.dot(b1);
        float c2 = rel.dot(b2);
        output.push_back({i, c1, c2});
      }
    }
  } else {
    intersect(bvh, segment, output, node.left_child_index);
    intersect(bvh, segment, output, node.right_child_index);
  }
}

size_t number_of_intersected_triangles(const BVH::BVH &bvh,
                                       const BVH::Segment3D &segment,
                                       size_t node_index = 0) {
  const BVH::Node &node = bvh.nodes[node_index];
  if (!do_segment_intersect_aabb(segment, node.bounding_box))
    return 0;
  if (node.is_leaf()) {
    size_t n = 0;
    for (size_t i = node.first_primitive_index; i <= node.last_primitive_index;
         i++) {
      n += do_segment_intersect_triangle(segment, bvh.triangles[i]);
    }
    return n;
  } else {
    return number_of_intersected_triangles(bvh, segment,
                                           node.left_child_index) +
           number_of_intersected_triangles(bvh, segment,
                                           node.right_child_index);
  }
}