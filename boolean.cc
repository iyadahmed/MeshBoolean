#include <iostream>

#include "bvh2.hh"
#include "bvh_point_query.hh"
#include "meshio.hh"
#include "timers.hh"

#include <valgrind/callgrind.h>

using namespace meshio::stl;

int main(int argc, const char *argv[]) {

  if (argc != 2) {
    puts("Usage: boolean file.stl");
    return 1;
  }

  const char *filename = argv[1];

  BinaryFileReader stl_file_reader(filename);
  BVH2 bvh;

  size_t tris_num = stl_file_reader.get_reported_number_of_triangles();
  for (size_t i = 0; i < tris_num; i++) {
    auto t = stl_file_reader.read_next_triangle();
    bvh.triangles.push_back({t.vertices[0], t.vertices[1], t.vertices[2]});
  }
  std::cout << "Number of triangles = " << bvh.triangles.size() << std::endl;

  Timer timer;

  bvh.update_tree();

  timer.tock("Building BVH");

  std::vector<Vec3> points;
  //  std::vector<BVH2::Intersection_Point> points;

  CALLGRIND_START_INSTRUMENTATION;
  CALLGRIND_TOGGLE_COLLECT;

  timer.tick();
  for (const auto &t : bvh.triangles) {
    //    BVH2::Segment s1{t.verts[0], t.verts[1]};
    //    BVH2::Segment s2{t.verts[1], t.verts[2]};
    //    BVH2::Segment s3{t.verts[2], t.verts[0]};
    //
    //    bvh.intersect(s1, points);
    //    bvh.intersect(s2, points);
    //    bvh.intersect(s3, points);
    for (const auto &v : t.verts) {
      size_t i = closest_triangle(bvh, v, .01);
      if (i == INVALID_INDEX)
        points.push_back(v);
    }
  }
  timer.tock();

  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_STOP_INSTRUMENTATION;

  std::cout << "Number of intersection points = " << points.size() << std::endl;

  //  std::cout << "Number of leaf nodes = " << bvh.count_leaf_nodes() << std::endl;
  //
  //  BVH2::Segment s;
  //  s[0] = {0, 0, 2};
  //  s[1] = {0, 0, 1};
  //  std::cout << "Number of triangles intersecting segment " << bvh.number_of_intersected_triangles(s) << std::endl;

  // TODO: intersect all mesh edges with the mesh itself using the BVH and
  // collect intersection points
  // TODO: re-triangulate surface
  // TODO: export

  return 0;
}