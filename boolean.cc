#include <iostream>

#include "bvh2.hh"
#include "meshio.hh"
#include "timers.hh"

using namespace meshio::stl;

int main(int argc, const char *argv[]) {

  if (argc != 2) {
    puts("Usage: boolean file.stl");
  }

  const char *filename = argv[1];

  BinaryFileReader stl_file_reader(filename);
  std::vector<BVH2::Triangle> bvh_tris;
  size_t tris_num = stl_file_reader.get_reported_number_of_triangles();
  for (size_t i = 0; i < tris_num; i++) {
    auto t = stl_file_reader.read_next_triangle();
    bvh_tris.push_back({t.vertices[0], t.vertices[1], t.vertices[2]});
  }
  std::cout << "Number of triangles = " << bvh_tris.size() << std::endl;

  Timer timer;
  BVH2 bvh(bvh_tris);
  timer.tock("Building BVH");
  std::cout << "Number of leaf nodes = " << bvh.count_leaf_nodes(0)
            << std::endl;

  // TODO: perform self intersection using the BVH
  // TODO: re-triangulate surface
  // TODO: export

  return 0;
}