#include <iostream>

#include "bvh2.hh"
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

  CALLGRIND_START_INSTRUMENTATION;
  CALLGRIND_TOGGLE_COLLECT;
  bvh.update_tree();
  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_STOP_INSTRUMENTATION;

  timer.tock("Building BVH");
  std::cout << "Number of leaf nodes = " << bvh.count_leaf_nodes() << std::endl;

  BVH2::Segment s;
  s[0] = {0, 0, 2};
  s[1] = {0, 0, 1};
  std::cout << "Number of triangles intersecting segment " << bvh.number_of_intersected_triangles(s) << std::endl;

  // TODO: intersect all mesh edges with the mesh itself using the BVH and
  // collect intersection points
  // TODO: re-triangulate surface
  // TODO: export

  return 0;
}