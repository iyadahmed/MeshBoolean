#pragma once

#include <limits>
#include <stack>
#include <stdexcept>

#include "bvh2.hh"

bool contains_point(const BVH2::AABB &aabb, const Vec3 &point, float margin) {
  return point.x > aabb.min.x - margin and point.y > aabb.min.y - margin and point.z > aabb.min.z - margin and
         point.x < aabb.max.x + margin and point.y < aabb.max.y + margin and point.z < aabb.max.z + margin;
}

constexpr size_t INVALID_INDEX = std::numeric_limits<size_t>::max();

size_t closest_triangle(const BVH2 &bvh, const Vec3 &point, float margin = 0.001f) {
  if (bvh.nodes_.empty()) {
    throw std::runtime_error("Empty BVH");
  }
  std::stack<size_t> nodes_stack;
  nodes_stack.push(0);

  // Find closest leaf node
  while (not nodes_stack.empty()) {
    size_t node_index = nodes_stack.top();
    nodes_stack.pop();
    const BVH2::Node &node = bvh.nodes_[node_index];
    if (not contains_point(node.bounding_box, point, margin)) {
      continue;
    }

    if (not node.is_leaf()) {
      nodes_stack.push(node.right_child_index);
      nodes_stack.push(node.left_child_index);
      continue;
    }

    for (size_t i = node.first_primitive_index; i < node.last_primitive_index; i++) {
      const BVH2::Triangle &triangle = bvh.triangles[i];
      Vec3 normal = triangle.calc_normal();
      float distance = (point - triangle.verts[0]).dot(normal);
      if (distance < margin) {
        return i;
      }
    }
  }

  return INVALID_INDEX;
}