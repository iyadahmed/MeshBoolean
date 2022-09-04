#include <cstdio>
#include <iostream>
#include <numeric>
#include <queue>
#include <vector>

#include "ball_tree2.hh"
#include "bvh.hh"
#include "meshio.hh"
#include "timers.hh"

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s <filename.stl>\n", argv[0]);
    return 1;
  }

  std::string filename(argv[1]);

  std::cout << "Processing file: " << filename << std::endl;

  auto tris = read_binary_stl_unchecked(filename);
  std::cout << "Number of triangles = " << tris.size() << std::endl;

  //  std::vector<BVH::Triangle> bvh_tris;
  //  for (const auto &t : tris) {
  //    bvh_tris.push_back({t.verts[0], t.verts[1], t.verts[2]});
  //  }

  std::vector<Vec3> vertices;
  for (const auto &t : tris) {
    for (const auto &v : t.verts) {
      vertices.push_back(v);
    }
  }

  //  std::vector<Vec3> vertices;
  //  vertices.push_back(Vec3(-294.6, -56.9, -174.3));
  //  vertices.push_back(Vec3(-251.1, -145.1, -48.2));

  //  Timer timer;
  Ball_Tree2 ball_tree(vertices);
  //  timer.tock("Constructing Ball Tree");
  //  std::cout << "Number of Ball Tree leaf nodes = "
  //            << ball_tree.count_leaf_nodes(0) << std::endl;

  //  std::cout << ball_tree.point_exists(vertices[0], .0001) << std::endl;

  //  std::queue<Vec3> Q;
  //  ball_tree.knn_search(Q, vertices[1], 2);
  //
  //  std::cout << "Vertices: " << std::endl;
  //  for (const auto &v : vertices) {
  //    std::cout << v << std::endl;
  //  }
  //
  //  std::cout << "Queue: " << std::endl;
  //  while (not Q.empty()) {
  //    std::cout << Q.front() << std::endl;
  //    Q.pop();
  //  }

  //  timer.tick();
  for (size_t i = 0; i < vertices.size() / 1000; i++) {
    if (not ball_tree.point_exists(vertices[i], .001)) {
      throw;
    }
  }
  //  timer.tock("");

  //  timer.tick();
  //  BVH bvh(bvh_tris);
  //  timer.tock("Building BVH");
  //  std::cout << "Number of BVH nodes = " << bvh.calc_number_of_nodes()
  //            << std::endl;

  return 0;
}