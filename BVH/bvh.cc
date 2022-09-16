#include <cassert>

#include "bvh.hh"
#include "common.hh"

namespace BVH {
void BVH::update_node_bounds(size_t node_index) {
  Node &node = nodes_[node_index];
  node.bounding_box.min = std::numeric_limits<float>::infinity();
  node.bounding_box.max = -1 * node.bounding_box.min;
  for (size_t i = node.first_primitive_index; i <= node.last_primitive_index;
       i++) {
    const AABB &bb = triangles[i].cached_bounding_box;
    node.bounding_box.min.min(bb.min);
    node.bounding_box.max.max(bb.max);
  }
}

size_t BVH::count_leaf_nodes(size_t node_index) const {
  if (node_index == INVALID_INDEX) {
    return 0;
  }
  Node const &node = nodes_[node_index];
  if (node.is_leaf()) {
    return 1;
  }
  return count_leaf_nodes(node.left_child_index) +
         count_leaf_nodes(node.right_child_index);
}

LeafInfo BVH::find_biggest_leaf(size_t node_index) const {
  if (node_index == INVALID_INDEX) {
    return LeafInfo{INVALID_INDEX, 0};
  }
  Node const &node = nodes_[node_index];
  if (node.is_leaf()) {
    return LeafInfo{node_index,
                    node.last_primitive_index - node.first_primitive_index + 1};
  }
  LeafInfo l1 = find_biggest_leaf(node.right_child_index);
  LeafInfo l2 = find_biggest_leaf(node.left_child_index);
  if (l1.num_tris > l2.num_tris) {
    return l1;
  }
  return l2;
}

size_t BVH::count_leaf_triangles(size_t node_index) const {
  if (node_index == INVALID_INDEX) {
    return 0;
  }
  Node const &node = nodes_[node_index];
  if (node.is_leaf()) {
    return node.last_primitive_index - node.first_primitive_index + 1;
  }
  return count_leaf_triangles(node.left_child_index) +
         count_leaf_triangles(node.right_child_index);
}

size_t BVH::get_new_node_index() {
  nodes_.emplace_back();
  return num_used_nodes_++;
}

void BVH::update_tree() {
  nodes_.reserve(2 * triangles.size());

  for (auto &t : triangles) {
    t.cached_centroid = t.calc_centroid();
    t.cached_bounding_box = t.calc_bounding_box();
  }

  size_t root_node_index = get_new_node_index();
  Node &root_node = nodes_[root_node_index];
  root_node.first_primitive_index = 0;
  root_node.last_primitive_index = triangles.size() - 1;
  build_tree(root_node_index);
  assert(count_leaf_triangles(0) == triangles.size());
  assert(nodes_.size() <= (2 * triangles.size()));
}

void BVH::build_tree(size_t parent_node_index) {
  Node &parent_node = nodes_[parent_node_index];

  assert(parent_node.first_primitive_index <= parent_node.last_primitive_index);
  assert(parent_node.first_primitive_index < triangles.size());
  assert(parent_node.last_primitive_index < triangles.size());

  update_node_bounds(parent_node_index);

  if (parent_node.first_primitive_index == parent_node.last_primitive_index) {
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
  size_t i = parent_node.first_primitive_index;
  size_t j = parent_node.last_primitive_index + 1;

  while (i < j) {
    if (triangles[i].cached_centroid[split_axis] < split_pos) {
      i++;
    } else {
      assert(j != 0);
      j--;
      std::swap(triangles[i], triangles[j]);
    }
  }

  if (i == parent_node.first_primitive_index ||
      i == parent_node.last_primitive_index + 1) {
    return;
  }

  size_t left_first_primitive_index = parent_node.first_primitive_index;
  size_t left_last_primitive_index = i - 1;

  assert(left_last_primitive_index <= parent_node.last_primitive_index &&
         left_last_primitive_index >= parent_node.first_primitive_index);

  size_t right_first_primitive_index = left_last_primitive_index + 1;
  size_t right_last_primitive_index = parent_node.last_primitive_index;

  assert(right_first_primitive_index <= parent_node.last_primitive_index &&
         right_first_primitive_index >= parent_node.first_primitive_index);

  parent_node.left_child_index = get_new_node_index();
  Node &left_node = nodes_[parent_node.left_child_index];
  left_node.first_primitive_index = left_first_primitive_index;
  left_node.last_primitive_index = left_last_primitive_index;

  parent_node.right_child_index = get_new_node_index();
  Node &right_node = nodes_[parent_node.right_child_index];
  right_node.first_primitive_index = right_first_primitive_index;
  right_node.last_primitive_index = right_last_primitive_index;

  build_tree(parent_node.left_child_index);
  build_tree(parent_node.right_child_index);
}
} // namespace BVH