#include <cassert>

#include "bvh.hh"
#include "common.hh"

namespace BVH {
void BVH::update_node_bounds(uint32_t node_index) {
  Node &node = nodes_[node_index];
  node.bounding_box.min = std::numeric_limits<float>::infinity();
  node.bounding_box.max = -1 * node.bounding_box.min;
  for (size_t i = node.first_primitive_index;
       i < (node.first_primitive_index + node.number_of_primitives); i++) {
    const AABB &bb = triangles[i].cached_bounding_box;
    node.bounding_box.min.min(bb.min);
    node.bounding_box.max.max(bb.max);
  }
}

uint32_t BVH::count_leaf_nodes(uint32_t node_index) const {
  Node const &node = nodes_[node_index];
  if (node.is_leaf()) {
    return 1;
  }
  return count_leaf_nodes(node.left_child_index) +
         count_leaf_nodes(node.left_child_index + 1);
}

LeafInfo BVH::find_biggest_leaf(uint32_t node_index) const {
  Node const &node = nodes_[node_index];
  if (node.is_leaf()) {
    return LeafInfo{node_index, node.number_of_primitives};
  }
  LeafInfo l2 = find_biggest_leaf(node.left_child_index);
  LeafInfo l1 = find_biggest_leaf(node.left_child_index + 1);
  if (l1.num_tris > l2.num_tris) {
    return l1;
  }
  return l2;
}

uint32_t BVH::count_leaf_triangles(uint32_t node_index) const {
  Node const &node = nodes_[node_index];
  if (node.is_leaf()) {
    return node.number_of_primitives;
  }
  return count_leaf_triangles(node.left_child_index) +
         count_leaf_triangles(node.left_child_index + 1);
}

uint32_t BVH::get_new_node_index() {
  nodes_.emplace_back();
  return num_used_nodes_++;
}

void BVH::update_tree() {
  nodes_.reserve(2 * triangles.size());

  for (auto &t : triangles) {
    t.cached_bounding_box = t.calc_bounding_box();
    t.cached_centroid =
        (.5 * t.cached_bounding_box.max + .5 * t.cached_bounding_box.min);
  }

  uint32_t root_node_index = get_new_node_index();
  Node &root_node = nodes_[root_node_index];
  root_node.first_primitive_index = 0;
  root_node.number_of_primitives = triangles.size();
  build_tree(root_node_index);
  assert(count_leaf_triangles(0) == triangles.size());
  assert(nodes_.size() <= (2 * triangles.size()));
}

void BVH::build_tree(uint32_t parent_node_index) {
  Node &parent_node = nodes_[parent_node_index];

  assert(parent_node.first_primitive_index >= 0);
  assert(parent_node.first_primitive_index < triangles.size());
  assert(parent_node.first_primitive_index + parent_node.number_of_primitives <=
         triangles.size());

  update_node_bounds(parent_node_index);

  if (parent_node.number_of_primitives < 2) {
    return;
  }

  Vec3 extent = parent_node.bounding_box.max - parent_node.bounding_box.min;
  size_t split_axis = 0;
  if (extent[1] > extent[0]) {
    split_axis = 1;
  }
  if (extent[2] > extent[split_axis]) {
    split_axis = 2;
  }

  float split_pos =
      parent_node.bounding_box.min[split_axis] + extent[split_axis] / 2;

  // In-place partition
  uint32_t i = parent_node.first_primitive_index;
  uint32_t j = i + parent_node.number_of_primitives;

  while (i < j) {
    if (triangles[i].cached_centroid[split_axis] < split_pos) {
      i++;
    } else {
      assert(j != 0);
      j--;
      std::swap(triangles[i], triangles[j]);
    }
  }

  uint32_t left_first_primitive_index = parent_node.first_primitive_index;
  uint32_t left_num_primitives = i - left_first_primitive_index;

  if (left_num_primitives == 0 ||
      left_num_primitives == parent_node.number_of_primitives) {
    return;
  }

  uint32_t right_first_primitive_index = i;
  uint32_t right_num_primitives =
      parent_node.number_of_primitives - left_num_primitives;

  parent_node.number_of_primitives = 0;

  parent_node.left_child_index = get_new_node_index();
  Node &left_node = nodes_[parent_node.left_child_index];
  left_node.first_primitive_index = left_first_primitive_index;
  left_node.number_of_primitives = left_num_primitives;

  uint32_t right_child_index = get_new_node_index();
  Node &right_node = nodes_[right_child_index];
  right_node.first_primitive_index = right_first_primitive_index;
  right_node.number_of_primitives = right_num_primitives;

  build_tree(parent_node.left_child_index);
  build_tree(parent_node.left_child_index + 1);
}
} // namespace BVH