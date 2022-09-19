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
    size_t first_point_index, last_point_index;

    static constexpr size_t INVALID_INDEX = std::numeric_limits<size_t>::max();

    Node() {
      center = Vec3(0, 0, 0);
      radius = 0.0f;
      left_child_index = right_child_index = INVALID_INDEX;
      first_point_index = last_point_index = INVALID_INDEX;
    }

    bool is_leaf() const {
      return left_child_index == INVALID_INDEX && right_child_index == INVALID_INDEX;
    }
  };

private:
  size_t num_used_nodes_;
  std::vector<Node> nodes_;

public:
  explicit Ball_Tree2(std::vector<Vec3> &points) {
    num_used_nodes_ = 0;
    nodes_.resize(2 * points.size());
    construct_ball_tree(0, points, 0, points.size() - 1);
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

  void construct_ball_tree(size_t parent_node_index, std::vector<Vec3> &points, size_t first_index,
                           size_t last_index) {
    // TODO: refactor to be non recursive
    tassert(first_index <= last_index);

    Node &parent_node = nodes_[parent_node_index];

    parent_node.first_point_index = first_index;
    parent_node.last_point_index = last_index;
    RunningStat<Vec3> running_stat;
    for (size_t i = first_index; i <= last_index; i++) {
      running_stat.push(points[i]);
    }
    if (parent_node_index == 0) {
      parent_node.center = running_stat.get_mean();
    }
    parent_node.radius = std::numeric_limits<float>::min();
    for (size_t i = first_index; i <= last_index; i++) {
      float d = distance(points[i], parent_node.center);
      parent_node.radius = std::max(parent_node.radius, d);
    }

    Vec3 far_point1 = points[first_index];
    for (size_t i = first_index + 1; i <= last_index; i++) {
      float new_d = distance(points[i], parent_node.center);
      float old_d = distance(far_point1, parent_node.center);
      if (new_d > old_d) {
        far_point1 = points[i];
      }
    }

    Vec3 far_point2 = points[first_index];
    for (size_t i = first_index + 1; i <= last_index; i++) {
      float new_d = distance(points[i], far_point1);
      float old_d = distance(far_point2, far_point1);
      if (new_d > old_d) {
        far_point2 = points[i];
      }
    }

    // Partition into two clusters
    Vec3 V = far_point2 - far_point1;
    Vec3 P = (far_point2 + far_point1) / 2;
    size_t partition_start = first_index;
    size_t partition_end = last_index;
    while (partition_start < partition_end) {
      if ((points[partition_start] - P).dot(V) < 0.0f) {
        partition_start++;
      } else {
        std::swap(points[partition_start], points[partition_end]);
        tassert(partition_end != 0);
        partition_end--;
      }
    }

    if (partition_start == first_index || partition_end == last_index) {
      parent_node.center = running_stat.get_mean();
      return;
    }

    size_t left_first_index = first_index;
    size_t left_last_index = partition_end;
    tassert(left_first_index <= left_last_index);

    size_t right_first_index = left_last_index + 1;
    size_t right_last_index = last_index;
    tassert(right_first_index <= right_last_index);

    size_t left_node_index = get_new_node_index();
    size_t right_node_index = get_new_node_index();

    parent_node.left_child_index = left_node_index;
    parent_node.right_child_index = right_node_index;

    Node &left_node = nodes_[left_node_index];
    Node &right_node = nodes_[right_node_index];

    left_node.center = first_cluster_centroid;
    right_node.center = second_cluster_centroid;

    construct_ball_tree(left_node_index, points, left_first_index, left_last_index);
    construct_ball_tree(right_node_index, points, right_first_index, right_last_index);
  }

  //  void knn_search(std::queue<Vec3> &output, Vec3 const &query, size_t k) {
  //    knn_search(output, query, k, nodes_[0]);
  //  }

  // FIXME: copy points inside the structure to avoid passing it like this
  bool point_exists(Vec3 const &query, float radius, std::vector<Vec3> const &points) const {
    return point_exists(query, nodes_[0], radius, points);
  }

private:
  bool point_exists(Vec3 const &query, Node const &node, float radius,
                    std::vector<Vec3> const &points) const {
    if (node.is_leaf()) {
      for (size_t i = node.first_point_index; i <= node.last_point_index; i++) {
        if (distance(query, points[i]) < radius) {
          return true;
        }
      }
      return false;
    }

    //    float d = distance(query, node.center);
    //    if (d > (node.radius + radius)) {
    //      // "If outside ball, return false"
    //      // NOTE: this assumes leaf nodes have 0 radius
    //      return false;
    //    }
    //    if (node.is_leaf()) {
    //      for (size_t i = node.first_point_index; i <= node.last_point_index;
    //      i++) {
    //        if (distance(query, points[i]) < radius) {
    //          return true;
    //        }
    //      }
    //      return false;
    //    }
    tassert(node.left_child_index != Node::INVALID_INDEX);
    tassert(node.right_child_index != Node::INVALID_INDEX);
    Node const &left = nodes_[node.left_child_index];
    Node const &right = nodes_[node.right_child_index];

    float dL = distance(left.center, query);
    float dR = distance(right.center, query);
    if (dL < dR) {
      return point_exists(query, left, radius, points);
    } else {
      return point_exists(query, right, radius, points);
    }
  }

  // FIXME: implement KNN search correctly

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