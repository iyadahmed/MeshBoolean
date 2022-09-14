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
  struct AABB {
    Vec3 max, min;
  };

  struct Node {
    AABB bounding_box;
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

  struct Barycentric_Info {
    // https://blackpawn.com/texts/pointinpoly/
    Vec3 triangle_v0;
    Vec3 v0, v1;
    float dot00, dot01, dot11, inv_denom;

    explicit Barycentric_Info(const Triangle &triangle) {
      triangle_v0 = triangle[0];
      v0 = triangle[2] - triangle[0];
      v1 = triangle[1] - triangle[0];
      dot00 = dot(v0, v0);
      dot01 = dot(v0, v1);
      dot11 = dot(v1, v1);
      inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);
    }

    bool is_inside_triangle(Vec3 const &point) const {
      Vec3 v2 = point - triangle_v0;
      float dot02 = dot(v0, v2);
      float dot12 = dot(v1, v2);
      float u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
      float v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
      return (u >= 0) && (v >= 0) && (u + v < 1);
    }
  };

private:
  size_t num_used_nodes_ = 0;
  std::vector<Node> nodes_;

public:
  std::vector<Triangle> triangles;

public:
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

  size_t count_leaf_nodes(size_t node_index = 0) const {
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

  size_t cound_leaf_triangles(size_t node_index = 0) const {
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

  size_t number_of_intersected_triangles(const Segment &segment,
                                         size_t node_index = 0) {
    Node &node = nodes_[node_index];
    if (!do_segment_intersect_aabb(segment, node.bounding_box))
      return 0;
    if (node.is_leaf()) {
      size_t n = 0;
      for (size_t i = node.first_primitive_index; i < node.last_primitive_index;
           i++) {
        n += do_segment_intersect_triangle(segment, triangles[i]);
      }
      return n;
    } else {
      return number_of_intersected_triangles(segment, node.left_child_index) +
             number_of_intersected_triangles(segment, node.right_child_index);
    }
  }

private:
  static Vec3 centroid(const Triangle &t) { return (t[0] + t[1] + t[2]) / 3; }

  static Vec3 calc_normal(const Triangle &t) {
    return (t[1] - t[0]).cross(t[2] - t[0]).normalized();
  }

  static int signof(const float &value, const float &&epsilon) {
    if (value > epsilon) {
      return 1;
    }
    if (value < epsilon) {
      return -1;
    }
    return 0;
  }

  size_t get_new_node_index() {
    tassert(num_used_nodes_ < nodes_.size());
    return num_used_nodes_++;
  }

  void update_node_bounds(size_t node_index) {
    Node &node = nodes_[node_index];
    node.bounding_box.min = std::numeric_limits<float>::infinity();
    node.bounding_box.max = -1 * node.bounding_box.min;
    for (size_t i = node.first_primitive_index; i < node.last_primitive_index;
         i++) {
      for (Vec3 const &v : triangles[i]) {
        tassert(std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z));
        node.bounding_box.min.min(v);
        node.bounding_box.max.max(v);
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
  bool do_segment_intersect_aabb(const Segment &segment, const AABB &aabb) {

    const Vec3 &p1 = segment[0];
    const Vec3 &p2 = segment[1];

    constexpr float EPSILON = std::numeric_limits<float>::epsilon() * 100;
    Vec3 d = (p2 - p1) * 0.5f;
    Vec3 e = (aabb.max - aabb.min) * 0.5f;
    Vec3 c = p1 + d - (aabb.min + aabb.max) * 0.5f;
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

  static bool do_segment_intersect_triangle(const Segment &segment,
                                            const Triangle &triangle) {
    Vec3 s1 = segment[0] - triangle[0];
    Vec3 s2 = segment[1] - triangle[0];
    Vec3 triangle_normal = calc_normal(triangle);

    float d1 = std::abs(triangle_normal.dot(s1));
    float d2 = std::abs(triangle_normal.dot(s2));

    int sign1 = signof(d1, 0.0001f);
    int sign2 = signof(d2, 0.0001f);

    if ((sign1 == sign2) && (sign1 != 0)) {
      // Both segment points are on the same side of the triangle supporting
      // plane, no intersection is possible
      //      return {{}, 0};
      return false;
    }

    Barycentric_Info barycentric_info(triangle);

    if ((sign1 == 0) && (sign2 == 0)) {
      // Coplanar case
      /* TODO:
      1. if any point of the two segment points are inside triangle, include
      it in the intersection result
      2. if both points are inside triangle return that result
      3. if not, intersect segment with all triangle segments
      4. note there might be no intersections, if the segment doesn't
      intersect any of triangle segments, nor does it have any point inside
      triangle
      */
      return false;
    }

    // Non-coplanar case
    // Intersect segment supporting ray with triangle supporting plane
    // https://stackoverflow.com/a/23976134/8094047
    Vec3 ray_direction = (segment[1] - segment[0]).normalized();
    float denom = triangle_normal.dot(ray_direction);
    float t = (triangle[0] - segment[0]).dot(triangle_normal) / denom;
    Vec3 intersection_point = t * ray_direction + segment[0];

    if (barycentric_info.is_inside_triangle(intersection_point)) {
      //      return {{intersection_point, {}}, 1};
      return true;
    }

    //    return {{}, 0};
    return false;
  }
};