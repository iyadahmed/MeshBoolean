#include <immintrin.h>

#include "query_point.hh"

static bool is_point_outside_aabb(const BVH::AABB &aabb, const Vec3 &point) {
  assert((point.x > aabb.max.x or point.y > aabb.max.y or
          point.z > aabb.max.z or point.x < aabb.min.x or
          point.y < aabb.min.y or point.z < aabb.min.z) ==
         (aabb.min.any_gt(point) or point.any_gt(aabb.max)));

  return aabb.min.any_gt(point) or point.any_gt(aabb.max);
}

namespace BVH {

bool contains_point(const BVH &bvh, const Vec3 &point, size_t node_index) {
  const Node &node = bvh.nodes[node_index];
  if (is_point_outside_aabb(node.bounding_box, point)) {
    return false;
  }

  if (node.is_leaf()) {
    for (size_t i = node.first_primitive_index;
         i < (node.first_primitive_index + node.number_of_primitives); i++) {
      const Triangle &tri = bvh.triangles[i];
      for (const auto &v : tri.verts) {
        if (distance_squared(v, point) < .0001f * .0001f) {
          return true;
        }
      }
    }

    return false;
  } else {
    return contains_point(bvh, point, node.left_child_index) ||
           contains_point(bvh, point, node.left_child_index + 1);
  }
}
} // namespace BVH