#include "query_point.hh"

static bool contains_point(const BVH::AABB &aabb, const Vec3 &point) {
  return (point.x >= aabb.min.x) && (point.y >= aabb.min.y) &&
         (point.z >= aabb.min.z) && (point.x <= aabb.max.x) &&
         (point.y <= aabb.max.y) && (point.z <= aabb.max.z);
}

namespace BVH {

bool contains_point(const BVH &bvh, const Vec3 &point, size_t node_index) {
  const Node &node = bvh.nodes[node_index];
  if (not contains_point(node.bounding_box, point)) {
    return false;
  }

  if (node.is_leaf()) {
    for (size_t i = node.first_primitive_index;
         i < (node.first_primitive_index + node.number_of_primitives); i++) {
      const Triangle &tri = bvh.triangles[i];
      for (const auto &v : tri.verts) {
        if (distance(v, point) < .0001f) {
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