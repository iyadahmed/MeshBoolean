#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "common.hh"
#include "non_copyable.hh"
#include "vec3.hh"

inline bool all_greater_than(const Vec3 &a, const Vec3 &b) {
  return (a.x > b.x) && (a.y > b.y) && (a.z > b.z);
}

inline bool all_less_than(const Vec3 &a, const Vec3 &b) {
  return (a.x < b.x) && (a.y < b.y) && (a.z < b.z);
}

int signof(const float &value, const float &&epsilon) {
  if (value > epsilon) {
    return 1;
  }
  if (value < epsilon) {
    return -1;
  }
  return 0;
}

class BVH : public NonCopyable {

public:
  struct BBox {
    Vec3 max, min;
  };

  struct Ray {
    Vec3 origin, direction;
    float t = INFINITY;
  };

  struct Triangle {
    std::array<Vec3, 3> points;
    Vec3 centroid_cached;

    Vec3 calc_normal() const {
      return (points[1] - points[0]).cross(points[2] - points[0]).normalized();
    }
  };

  struct Barycentric_Info {
    // https://blackpawn.com/texts/pointinpoly/
    Vec3 triangle_v0;
    Vec3 v0, v1;
    float dot00, dot01, dot11, inv_denom;

    explicit Barycentric_Info(Triangle const &triangle) {
      triangle_v0 = triangle.points[0];
      v0 = triangle.points[2] - triangle.points[0];
      v1 = triangle.points[1] - triangle.points[0];
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

  struct Segment_Triangle_Intersection_Result {
    // Array to hold maximum number of resulting points
    std::array<Vec3, 2> points;
    size_t points_num = 0;
  };

  struct Segment {
    std::array<Vec3, 2> verts;

    BBox calc_bbox() const {
      BBox bbox;
      bbox.max = Vec3::max(verts[0], verts[1]);
      bbox.min = Vec3::min(verts[0], verts[1]);
      return bbox;
    }

    Segment_Triangle_Intersection_Result
    intersect_triangle(const Triangle &triangle) const {
      Vec3 s1 = verts[0] - triangle.points[0];
      Vec3 s2 = verts[1] - triangle.points[0];
      Vec3 triangle_normal = triangle.calc_normal();

      float d1 = std::abs(triangle_normal.dot(s1));
      float d2 = std::abs(triangle_normal.dot(s2));

      int sign1 = signof(d1, 0.0001f);
      int sign2 = signof(d2, 0.0001f);

      if ((sign1 == sign2) && (sign1 != 0)) {
        // Both segment points are on the same side of the triangle supporting
        // plane, no intersection is possible
        return {{}, 0};
      }

      Barycentric_Info binfo(triangle);

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
      }

      // Non-coplanar case
      // Intersect segment supporting ray with triangle supporting plane
      // https://stackoverflow.com/a/23976134/8094047
      Vec3 ray_direction = (verts[1] - verts[0]).normalized();
      float denom = triangle_normal.dot(ray_direction);
      float t = (triangle.points[0] - verts[0]).dot(triangle_normal) / denom;
      Vec3 intersection_point = t * ray_direction + verts[0];

      if (binfo.is_inside_triangle(intersection_point)) {
        return {{intersection_point, {}}, 1};
      }

      return {{}, 0};
    }
  };

  struct Node {
    Node *left = nullptr, *right = nullptr;
    BBox bbox;
    std::vector<Triangle>::iterator start, end;

    bool is_leaf() const { return (left == nullptr) && (right == nullptr); };

    size_t triangles_count() const { return std::distance(start, end); }

    bool does_overlap(const Node &other) const {
      return all_greater_than(bbox.max, other.bbox.min) &&
             all_less_than(bbox.min, other.bbox.max);
    }

    bool does_overlap(const BBox &other_bbox) const {
      return all_greater_than(bbox.max, other_bbox.min) &&
             all_less_than(bbox.max, other_bbox.max);
    }
  };

private:
  Node *nodes;
  int num_used_nodes;

  static void recalc_bounds(Node *node, const std::vector<Triangle> &tris) {
    node->bbox.max = -INFINITY;
    node->bbox.min = INFINITY;

    tassert(node->start >= tris.begin());
    tassert(node->start <= tris.end());
    tassert(node->end >= tris.begin());
    tassert(node->end <= tris.end());

    for (auto tri_iter = node->start; tri_iter < node->end; tri_iter++) {
      for (int i = 0; i < 3; i++) {
        node->bbox.max.max(tri_iter->points[i]);
        node->bbox.min.min(tri_iter->points[i]);
      }
    }
  }

  void subdivide(Node *root, std::vector<Triangle> &tris) {
    if (root->triangles_count() <= 2) {
      return;
    }
    Vec3 dims = root->bbox.max - root->bbox.min;
    int split_axis = 0;
    if (dims.y < dims.x) {
      split_axis = 1;
    }
    if (dims.z > dims[split_axis]) {
      split_axis = 2;
    }
    float split_pos = root->bbox.min[split_axis] + dims[split_axis] * .5f;

    auto it = std::partition(root->start, root->end, [=](const Triangle &t) {
      return t.centroid_cached[split_axis] < split_pos;
    });

    if ((it == root->start) || (it == root->end)) {
      // abort split
      return;
    }

    Node *left = nodes + (num_used_nodes++);
    left->start = root->start;
    left->end = it;
    recalc_bounds(left, tris);
    left->left = left->right = nullptr;

    Node *right = nodes + (num_used_nodes++);
    right->start = it;
    right->end = root->end;
    recalc_bounds(right, tris);
    right->left = right->right = nullptr;

    root->left = left;
    root->right = right;

    subdivide(left, tris);
    subdivide(right, tris);
  }

  int calc_number_of_nodes_(const Node *root) const {
    if (root == nullptr) {
      return 0;
    }
    return 1 + calc_number_of_nodes_(root->left) +
           calc_number_of_nodes_(root->right);
  };

  void self_intersection_core(const Segment &segment, const Node *node) {
    if (node == nullptr) {
      return;
    }
    if (!node->does_overlap(segment.calc_bbox())) {
      return;
    }
    if (node->is_leaf()) {
      for (auto tri_iter = node->start; tri_iter < node->end; tri_iter++) {
        self_intersection_segment_triangle_intersection_handler(
            segment.intersect_triangle(*tri_iter), tri_iter);
      }
    }
    self_intersection_core(segment, node->left);
    self_intersection_core(segment, node->right);
  }

  struct Intersection_Point_Polar {
    std::vector<Triangle>::iterator tri_iter;
    float angle;
    float distance_squared;
  };

  std::vector<Intersection_Point_Polar> self_intersection_points;

  void self_intersection_segment_triangle_intersection_handler(
      Segment_Triangle_Intersection_Result const &intersection,
      std::vector<Triangle>::iterator tri_iter) {
    for (size_t i = 0; i < intersection.points_num; i++) {
      Vec3 p3d = intersection.points[i] - tri_iter->points[0];
      Vec3 basis1 = tri_iter->points[1] - tri_iter->points[0];
      Vec3 basis2 = tri_iter->points[2] - tri_iter->points[0];
      float x = p3d.dot(basis1);
      float y = p3d.dot(basis2);
      float angle = std::atan2(y, x);
      float distance_squared = x * x + y * y;
      self_intersection_points.push_back({tri_iter, angle, distance_squared});
    }
  }

public:
  BVH(std::vector<Triangle> &tris) {
    if (tris.empty()) {
      return;
    }
    nodes = new Node[2 * tris.size() - 1];

    for (Triangle &t : tris) {
      t.centroid_cached = (t.points[0] + t.points[1] + t.points[2]) / 3;
    }

    Node *root = nodes;
    root->start = tris.begin();
    root->end = tris.end();
    root->left = root->right = nullptr;
    recalc_bounds(root, tris);
    num_used_nodes = 1;
    subdivide(root, tris);
  }

  int calc_number_of_nodes() const { return calc_number_of_nodes_(nodes); }

  ~BVH() { delete[] nodes; }
};

void intersect_ray_tri(BVH::Ray &ray, const BVH::Triangle &tri) {
  const Vec3 edge1 = tri.points[1] - tri.points[0];
  const Vec3 edge2 = tri.points[2] - tri.points[0];
  const Vec3 h = cross(ray.direction, edge2);
  const float a = dot(edge1, h);
  if (a > -0.0001f && a < 0.0001f) {
    return; // ray parallel to triangle
  }
  const float f = 1 / a;
  const Vec3 s = ray.origin - tri.points[0];
  const float u = f * dot(s, h);
  if (u < 0 || u > 1) {
    return;
  }
  const Vec3 q = cross(s, edge1);
  const float v = f * dot(ray.direction, q);
  if (v < 0 || u + v > 1) {
    return;
  }
  const float t = f * dot(edge2, q);
  if (t > 0.0001f) {
    ray.t = std::min(ray.t, t);
  }
}

bool intersect_ray_aabb(const BVH::Ray &ray, const Vec3 &bmin,
                        const Vec3 &bmax) {
  float tx1 = (bmin.x - ray.origin.x) / ray.direction.x;
  float tx2 = (bmax.x - ray.origin.x) / ray.direction.x;
  float tmin = std::min(tx1, tx2);
  float tmax = std::max(tx1, tx2);
  float ty1 = (bmin.y - ray.origin.y) / ray.direction.y;
  float ty2 = (bmax.y - ray.origin.y) / ray.direction.y;
  tmin = std::max(tmin, std::min(ty1, ty2));
  tmax = std::min(tmax, std::max(ty1, ty2));
  float tz1 = (bmin.z - ray.origin.z) / ray.direction.z;
  float tz2 = (bmax.z - ray.origin.z) / ray.direction.z;
  tmin = std::max(tmin, std::min(tz1, tz2));
  tmax = std::min(tmax, std::max(tz1, tz2));
  return tmax >= tmin && tmin < ray.t && tmax > 0;
}
