#include <iostream>
#include <vector>

#include "bvh.hh"
#include "meshio.hh"

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s <filename.stl>\n", argv[0]);
    return 1;
  }

  auto tris = read_binary_stl_unchecked(argv[1]);
  std::cout << tris.size() << std::endl;

  std::vector<BVH::Triangle> bvh_tris;
  for (const auto &t : tris) {
    bvh_tris.push_back({t.verts[0], t.verts[1], t.verts[2]});
  }

  BVH bvh(bvh_tris);

  std::cout << bvh.calc_number_of_nodes() << std::endl;

  // Timer t;
  // BVH bvh(input_tris);
  // t.tock("Building BVH");
  // std::cout << "Number of BVH nodes = " << bvh.count() << std::endl;

  // t.tick();
  // bvh.self_overlap();
  // t.tock("Self overlap");

  return 0;
}