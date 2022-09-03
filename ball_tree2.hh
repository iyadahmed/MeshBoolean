#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <queue>
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
    if (node_index == Node::INVALID_INDEX) {
      return 0;
    }
    if (nodes_[node_index].is_leaf()) {
      return 1;
    }
    return count_leaf_nodes(nodes_[node_index].left_child_index) +
           count_leaf_nodes(nodes_[node_index].right_child_index);
  }

  void print_leaf_nodes(size_t node_index) {
    if (node_index == Node::INVALID_INDEX) {
      return;
    }
    if (nodes_[node_index].is_leaf()) {
      std::cout << nodes_[node_index].center << std::endl;
    }
    print_leaf_nodes(nodes_[node_index].left_child_index);
    print_leaf_nodes(nodes_[node_index].right_child_index);
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

    tassert(median_it >= points.begin() && median_it < points.end());

    new_node.center = *median_it;
    new_node.radius = std::numeric_limits<float>::min();
    for (size_t i = first_index; i <= last_index; i++) {
      new_node.radius =
          std::max(new_node.radius, (points[i] - (*median_it)).length());
    }

    new_node.left_child_index =
        construct_ball_tree(points, left_first_index, left_last_index);
    new_node.right_child_index =
        construct_ball_tree(points, right_first_index, right_last_index);
    return new_node_index;
  }

  //  void knn_search(std::queue<Vec3> &output, Vec3 const &query, size_t k) {
  //    knn_search(output, query, k, nodes_[0]);
  //  }

  bool point_exists(Vec3 const &query, float radius) const {
    return point_exists(query, nodes_[0], radius);
  }

private:
  bool point_exists(Vec3 const &query, Node const &node, float radius) const {
    float d = distance(query, node.center);
    if (d > (node.radius + radius)) {
      // NOTE: this assumes leaf nodes have 0 radius
      return false;
    } else {
      if (node.is_leaf()) {
        return true;
      } else {
        tassert(node.left_child_index != Node::INVALID_INDEX);
        tassert(node.right_child_index != Node::INVALID_INDEX);
        return point_exists(query, nodes_[node.left_child_index], radius) ||
               point_exists(query, nodes_[node.right_child_index], radius);
      }
    }
  }

  // FIXME: implement KNN search

  //  void knn_search(std::queue<Vec3> &output, Vec3 const &query, size_t k,
  //                  Node const &node) {
  //
  //    // TODO: refactor
  //    if (not output.empty()) {
  //
  //      if (distance(query, node.center) - node.radius >=
  //          distance(query, output.front())) {
  //        return;
  //      }
  //
  //    } else if (node.is_leaf()) {
  //
  //      if (output.empty()) {
  //        output.push(node.center);
  //
  //      } else {
  //
  //        if (distance(query, node.center) < distance(query, output.front()))
  //        {
  //          output.push(node.center);
  //          if (output.size() > k) {
  //            output.pop();
  //          }
  //        }
  //      }
  //
  //    } else {
  //      tassert(node.left_child_index != Node::INVALID_INDEX);
  //      tassert(node.right_child_index != Node::INVALID_INDEX);
  //      Node const &left_node = nodes_[node.left_child_index];
  //      Node const &right_node = nodes_[node.right_child_index];
  //      bool is_right_closer = distance(query, right_node.center) <
  //                             distance(query, left_node.center);
  //      if (is_right_closer) {
  //        knn_search(output, query, k, right_node);
  //        knn_search(output, query, k, left_node);
  //      } else {
  //        knn_search(output, query, k, left_node);
  //        knn_search(output, query, k, right_node);
  //      }
  //    }
  //  }
};