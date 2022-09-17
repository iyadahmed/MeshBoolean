#pragma once

#include <vector>

#include "../vec3.hh"
#include "aabb.hh"
#include "node.hh"
#include "triangle.hh"

namespace BVH {
struct LeafInfo {
  uint32_t index;
  uint32_t num_tris;
};

class BVH {
  // Reference:
  // https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
private:
  uint32_t num_used_nodes_ = 0;
  // TODO: Idea exponentially expanding array with max grow size
  std::vector<Node> nodes_;

  void build_tree(uint32_t parent_node_index);
  uint32_t get_new_node_index();
  void update_node_bounds(uint32_t node_index);

public:
  std::vector<Triangle> triangles;
  const std::vector<Node> &nodes = nodes_;

  void update_tree();
  uint32_t count_leaf_triangles(uint32_t node_index = 0) const;
  uint32_t count_leaf_nodes(uint32_t node_index = 0) const;
  LeafInfo find_biggest_leaf(uint32_t node_index = 0) const;
};
} // namespace BVH
