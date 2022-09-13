#pragma once

#include <array>
#include <cmath> // for std::isfinite
#include <vector>

#include "common.hh"
#include "vec3.hh"

// Initial inspiration
// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
class BVH2 {

public:
  struct Node {
    Vec3 bbmax, bbmin;
    size_t left_child_index, right_child_index;
    size_t first_primitive_index, last_primitive_index;

    static constexpr size_t INVALID_INDEX = std::numeric_limits<size_t>::max();

    Node() {
      left_child_index = right_child_index = INVALID_INDEX;
      first_primitive_index = last_primitive_index = INVALID_INDEX;
    }

    bool is_leaf() const {
      return left_child_index == INVALID_INDEX &&
             right_child_index == INVALID_INDEX;
    }
  };

  using Triangle = std::array<Vec3, 3>;
  using Segment = std::array<Vec3, 2>;

private:
  size_t num_used_nodes_;
  std::vector<Node> nodes_;

public:
  std::vector<Triangle> triangles;

public:
  BVH2() { num_used_nodes_ = 0; }

  void update_tree() {
    nodes_.resize(2 * triangles.size());
    size_t root_node_index = get_new_node_index();
    Node &root_node = nodes_[root_node_index];
    root_node.first_primitive_index = 0;
    root_node.last_primitive_index = triangles.size() - 1;
    update_node_bounds(root_node_index);
    subdivide(root_node_index);
    tassert(cound_leaf_triangles(0) == triangles.size());
  }

  size_t count_leaf_nodes(size_t node_index) const {
    if (node_index == Node::INVALID_INDEX) {
      return 0;
    }
    Node const &node = nodes_[node_index];
    if (node.is_leaf()) {
      return 1;
    }
    return count_leaf_nodes(node.left_child_index) +
           count_leaf_nodes(node.right_child_index);
  }

  size_t cound_leaf_triangles(size_t node_index) const {
    if (node_index == Node::INVALID_INDEX) {
      return 0;
    }
    Node const &node = nodes_[node_index];
    if (node.is_leaf()) {
      return node.last_primitive_index - node.first_primitive_index + 1;
    }
    return cound_leaf_triangles(node.left_child_index) +
           cound_leaf_triangles(node.right_child_index);
  }

  size_t intersect_segment(const Segment &segment, size_t node_index = 0) {
    Node &node = nodes_[node_index];
    if (!do_segment_intersect_aabb(segment[0], segment[1], node.bbmin,
                                   node.bbmax))
      return 0;
    if (node.is_leaf()) {
      return 1;
    } else {
      return intersect_segment(segment, node.left_child_index) +
             intersect_segment(segment, node.right_child_index);
    }
  }

private:
  static Vec3 centroid(const Triangle &t) { return (t[0] + t[1] + t[2]) / 3; }

  size_t get_new_node_index() {
    tassert(num_used_nodes_ < nodes_.size());
    return num_used_nodes_++;
  }

  void update_node_bounds(size_t node_index) {
    Node &node = nodes_[node_index];
    node.bbmin = std::numeric_limits<float>::infinity();
    node.bbmax = -1 * node.bbmin;
    for (size_t i = node.first_primitive_index; i < node.last_primitive_index;
         i++) {
      for (Vec3 const &v : triangles[i]) {
        tassert(std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z));
        node.bbmin.min(v);
        node.bbmax.max(v);
      }
    }
  }

  void subdivide(size_t parent_node_index) {

    Node &parent_node = nodes_[parent_node_index];

    tassert(parent_node.first_primitive_index <=
            parent_node.last_primitive_index);

    if (parent_node.first_primitive_index == parent_node.last_primitive_index) {
      return;
    }

    Vec3 extent = parent_node.bbmax - parent_node.bbmin;
    size_t split_axis = 0;
    if (extent[1] > extent[0]) {
      split_axis = 1;
    }
    if (extent[2] > extent[split_axis]) {
      split_axis = 2;
    }

    float split_pos = parent_node.bbmin[split_axis] + extent[split_axis] / 2;

    // In-place partition
    size_t partition_start = parent_node.first_primitive_index;
    size_t partition_end = parent_node.last_primitive_index;
    while (partition_start < partition_end) {
      if (centroid(triangles[partition_start])[split_axis] < split_pos) {
        partition_start++;
      } else {
        tassert(partition_end != 0);
        std::swap(triangles[partition_start], triangles[partition_end]);
        partition_end--;
      }
    }

    if (partition_start == parent_node.first_primitive_index ||
        partition_end == parent_node.last_primitive_index) {
      return;
    }

    size_t left_first_primitive_index = parent_node.first_primitive_index;
    size_t left_last_primitive_index = partition_end;

    size_t right_first_primitive_index = left_last_primitive_index + 1;
    size_t right_last_primitive_index = parent_node.last_primitive_index;

    parent_node.left_child_index = get_new_node_index();
    Node &left_node = nodes_[parent_node.left_child_index];
    left_node.first_primitive_index = left_first_primitive_index;
    left_node.last_primitive_index = left_last_primitive_index;
    update_node_bounds(parent_node.left_child_index);

    parent_node.right_child_index = get_new_node_index();
    Node &right_node = nodes_[parent_node.right_child_index];
    right_node.first_primitive_index = right_first_primitive_index;
    right_node.last_primitive_index = right_last_primitive_index;
    update_node_bounds(parent_node.right_child_index);

    subdivide(parent_node.left_child_index);
    subdivide(parent_node.right_child_index);
  }

  // From
  // https://gamedev.net/forums/topic/338987-aabb-line-segment-intersection-test/3209917/
  bool do_segment_intersect_aabb(const Vec3 &p1, const Vec3 &p2,
                                 const Vec3 &min, const Vec3 &max) {
    constexpr float EPSILON = std::numeric_limits<float>::epsilon() * 100;
    Vec3 d = (p2 - p1) * 0.5f;
    Vec3 e = (max - min) * 0.5f;
    Vec3 c = p1 + d - (min + max) * 0.5f;
    Vec3 ad = d.absolute();
    if (std::abs(c[0]) > e[0] + ad[0])
      return false;
    if (std::abs(c[1]) > e[1] + ad[1])
      return false;
    if (std::abs(c[2]) > e[2] + ad[2])
      return false;
    if (std::abs(d[1] * c[2] - d[2] * c[1]) >
        e[1] * ad[2] + e[2] * ad[1] + EPSILON)
      return false;
    if (std::abs(d[2] * c[0] - d[0] * c[2]) >
        e[2] * ad[0] + e[0] * ad[2] + EPSILON)
      return false;
    if (std::abs(d[0] * c[1] - d[1] * c[0]) >
        e[0] * ad[1] + e[1] * ad[0] + EPSILON)
      return false;
    return true;
  }
};