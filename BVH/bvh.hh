#pragma once

#include <vector>

#include "../vec3.hh"
#include "aabb.hh"
#include "node.hh"
#include "triangle.hh"

namespace BVH {
class BVH {
  // Reference:
  // https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
private:
  size_t num_used_nodes_ = 0;
  // TODO: Idea exponentially expanding array with max grow size
  std::vector<Node> nodes_;

  void subdivide(size_t parent_node_index);
  size_t get_new_node_index();
  void update_node_bounds(size_t node_index);

public:
  std::vector<Triangle> triangles;
  const std::vector<Node> &nodes = nodes_;

  void update_tree();
  size_t count_leaf_triangles(size_t node_index = 0) const;
  size_t count_leaf_nodes(size_t node_index = 0) const;
};
} // namespace BVH
