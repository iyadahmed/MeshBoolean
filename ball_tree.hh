#pragma once

#include <algorithm>
#include <vector>

#include "common.hh"
#include "vec3.hh"

class Ball_Tree {
private:
  struct Node {
    Vec3 pivot;
    float radius;
    size_t left, right;

    Node() {
      pivot = {};
      radius = 0.0f;
      left = right = std::numeric_limits<size_t>::max();
    }
  };

private:
  Node *nodes_;
  size_t cap_;
  size_t size_;
  std::vector<size_t> indices;
  //
  //  size_t create_new_node() {
  //    if (size_ == cap_) {
  //      // Allocate new nodes_ array
  //      cap_ *= 2;
  //      Ball_Tree_Node *new_nodes = new Ball_Tree_Node[cap_]{};
  //      // Copy old items
  //      for (size_t i = 0; i < size_; i++) {
  //        new_nodes[i] = nodes_[i];
  //      }
  //      // Free old memory and set pointer to new nodes_ array
  //      delete[] nodes_;
  //      nodes_ = new_nodes;
  //    }
  //    return size_++;
  //  }

public:
  explicit Ball_Tree(std::vector<Vec3> &points) {
    tassert(not points.empty());
    cap_ = 2 * points.size(); // Should hold enough nodes for binary tree
    size_ = 0;
    nodes_ = new Node[cap_]{};
    construct_ball_tree(points, 0, points.size() - 1);
  }

  ~Ball_Tree() { delete[] nodes_; }

  size_t calc_number_of_nodes(size_t node_index) {
    if (node_index == std::numeric_limits<size_t>::max()) {
      return 0;
    }
    return 1 + calc_number_of_nodes(nodes_[node_index].left) +
           calc_number_of_nodes(nodes_[node_index].right);
  }

private:
  size_t get_new_node_index() {
    tassert(size_ < cap_);
    return size_++;
  }

  size_t construct_ball_tree(std::vector<Vec3> &points, size_t first_index,
                             size_t last_index) {
    tassert(first_index <= last_index);
    if (last_index == first_index) {
      size_t leaf_node_index = get_new_node_index();
      Node &leaf_node = nodes_[leaf_node_index];
      leaf_node = {};
      leaf_node.pivot = points[first_index];
      return leaf_node_index;
    }

    // Calculate bounding box
    Vec3 bbmin = std::numeric_limits<float>::max();
    Vec3 bbmax = std::numeric_limits<float>::min();
    for (size_t i = first_index; i < last_index; i++) {
      bbmin.min(points[i]);
      bbmax.max(points[i]);
    }
    Vec3 bbdims = bbmax - bbmin;

    // Determine split axis and position
    size_t split_axis = 0;
    if (bbdims[1] > bbdims[0]) {
      split_axis = 1;
    }
    if (bbdims[2] > bbdims[split_axis]) {
      split_axis = 2;
    }
    float split_pos = bbmin[split_axis] + bbdims[split_axis] * .5f;

    size_t i = first_index;
    size_t k = last_index;
    while (i <= k) {
      if (points[i][split_axis] < split_pos) {
        i++;
      } else {
        std::swap(points[i], points[k]);
        k--;
      }
    }

    if (i == first_index || k == last_index) {
      return std::numeric_limits<size_t>::max();
    }

    size_t left_first_index = first_index;
    size_t left_last_index = k;
    size_t right_first_index = k + 1;
    size_t right_last_index = last_index;

    size_t new_node_index = get_new_node_index();
    tassert(new_node_index < cap_ && new_node_index >= 0);
    Node &new_node = nodes_[new_node_index];
    new_node = {};
    //    new_node.pivot = *central_point;
    // TODO: calculate radius
    new_node.radius = std::numeric_limits<size_t>::min();
    //    for (auto const &p : points) {
    //      new_node.radius =
    //          std::max(new_node.radius, (p - *central_point).length());
    //    }

    new_node.left =
        construct_ball_tree(points, left_first_index, left_last_index);
    new_node.right =
        construct_ball_tree(points, right_first_index, right_last_index);

    return new_node_index;
  }
};