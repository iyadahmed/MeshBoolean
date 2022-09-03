#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

#include "common.hh"
#include "statistics.hh"
#include "vec3.hh"

class Ball_Tree2 {
private:
  struct Node {
    Vec3 center;
    float radius;
    size_t left_child_index, right_child_index;

    static constexpr size_t INVALID_INDEX = std::numeric_limits<size_t>::max();

    Node() {
      center = Vec3(0, 0, 0);
      radius = 0.0f;
      left_child_index = right_child_index = INVALID_INDEX;
    }

    bool is_leaf() const {
      return left_child_index == INVALID_INDEX &&
             right_child_index == INVALID_INDEX;
    }
  };

private:
  size_t num_used_nodes_;
  std::vector<Node> nodes_;

public:
  explicit Ball_Tree2(std::vector<Vec3> &points) {
    num_used_nodes_ = 0;
    nodes_.resize(2 * points.size());
    construct_ball_tree(points, 0, points.size() - 1);
  }

  size_t get_new_node_index() {
    tassert(num_used_nodes_ < nodes_.size());
    return num_used_nodes_++;
  }

  size_t count_leaf_nodes(size_t node_index) {
    if (node_index == std::numeric_limits<size_t>::max()) {
      return 0;
    }
    if (nodes_[node_index].is_leaf()) {
      return 1;
    }
    return count_leaf_nodes(nodes_[node_index].left_child_index) +
           count_leaf_nodes(nodes_[node_index].right_child_index);
  }

  size_t construct_ball_tree(std::vector<Vec3> &points, size_t first_index,
                             size_t last_index) {
    // TODO: refactor to be non recursive
    tassert(first_index <= last_index);
    if (last_index == first_index) {
      size_t leaf_node_index = get_new_node_index();
      Node &leaf_node = nodes_[leaf_node_index];
      leaf_node = {};
      leaf_node.center = points[first_index];
      return leaf_node_index;
    }

    size_t n = last_index - first_index + 1;

    RunningStat<Vec3> running_stat;
    for (size_t i = first_index; i <= last_index; i++) {
      running_stat.push(points[i]);
    }
    auto variance = running_stat.get_variance();

    // Naive variance computation
    //    Vec3 mean(0.0f);
    //    for (size_t i = first_index; i <= last_index; i++) {
    //      mean += points[i] / n;
    //    }
    //    Vec3 variance(0.0f);
    //    for (size_t i = first_index; i <= last_index; i++) {
    //      variance += (points[i] - mean).elementwise_squared() / n;
    //    }

    size_t greatest_variance_axis = 0;
    if (variance[1] > variance[0]) {
      greatest_variance_axis = 1;
    }
    if (variance[2] > variance[greatest_variance_axis]) {
      greatest_variance_axis = 2;
    }

    auto first_it = points.begin() + first_index;
    auto one_past_end_it = points.begin() + last_index + 1;
    auto median_it = points.begin() + (n - 1) / 2;

    std::nth_element(first_it, median_it, one_past_end_it,
                     [=](const Vec3 &a, const Vec3 &b) {
                       return a[greatest_variance_axis] <
                              b[greatest_variance_axis];
                     });

    size_t left_first_index = first_index;
    size_t left_last_index = first_index + (n - 1) / 2;
    tassert(left_first_index <= left_last_index);

    size_t right_first_index = left_last_index + 1;
    size_t right_last_index = last_index;
    tassert(right_first_index <= right_last_index);

    size_t new_node_index = get_new_node_index();
    Node &new_node = nodes_[new_node_index];
    new_node.left_child_index =
        construct_ball_tree(points, left_first_index, left_last_index);
    new_node.right_child_index =
        construct_ball_tree(points, right_first_index, right_last_index);
    return new_node_index;
  }
};