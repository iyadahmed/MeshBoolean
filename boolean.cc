#include <iostream>

#include "meshio.hh"

using namespace meshio::stl;

int main(int argc, const char *argv[]) {

  if (argc != 2) {
    puts("Usage: boolean file.stl");
  }

  const char *filename = argv[1];

  auto tris = read_binary_stl_tris_unchecked(filename);
  std::cout << "Number of triangles = " << tris.size() << std::endl;

  // TODO: build BVH
  // TODO: perform self intersection using the BVH
  // TODO: re-triangulate surface
  // TODO: export

  return 0;
}