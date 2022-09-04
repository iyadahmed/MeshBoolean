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

  auto tris = read_binary_stl_tris_unchecked(filename);
  std::cout << "Number of triangles = " << tris.size() << std::endl;

  std::vector<BVH2::Triangle> bvh_tris;
  for (auto t : tris) {
    bvh_tris.push_back({t.vertices[0], t.vertices[1], t.vertices[2]});
  }

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