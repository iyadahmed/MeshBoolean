#include "query_point.hh"

static bool contains_point(const BVH::AABB &aabb, const Vec3 &point,
                           float margin) {
  return (point.x > aabb.min.x - margin) && (point.y > aabb.min.y - margin) &&
         (point.z > aabb.min.z - margin) && (point.x < aabb.max.x + margin) &&
         (point.y < aabb.max.y + margin) && (point.z < aabb.max.z + margin);
}

namespace BVH {
size_t closest_triangle(const BVH &bvh, const Vec3 &point, float max_distance) {
  if (bvh.nodes.empty()) {
    throw std::runtime_error("Empty BVH");
  }
  std::stack<size_t> nodes_stack;
  nodes_stack.push(0);

  // Find closest leaf node
  while (not nodes_stack.empty()) {
    size_t node_index = nodes_stack.top();
    nodes_stack.pop();
    const Node &node = bvh.nodes[node_index];
    if (not contains_point(node.bounding_box, point, max_distance)) {
      continue;
    }

    if (not node.is_leaf()) {
      nodes_stack.push(node.right_child_index);
      nodes_stack.push(node.left_child_index);
      continue;
    }

    for (size_t i = node.first_primitive_index; i <= node.last_primitive_index;
         i++) {
      const Triangle &triangle = bvh.triangles[i];
      Vec3 normal = triangle.calc_normal();
      float distance = (point - triangle.verts[0]).dot(normal);
      if (distance < max_distance) {
        return i;
      }
    }
  }

  return INVALID_INDEX;
}
} // namespace BVH