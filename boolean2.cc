#include <iostream>

#include "BVH/bvh.hh"
#include "BVH/query_point.hh"
#include "BVH/segment_bvh_intersection.hh"
#include "meshio/stl/stl_binary_writer.hh"
#include "meshio/tri_ascii_reader.hh"
#include "timers.hh"

#include <valgrind/callgrind.h>

int main(int argc, const char *argv[]) {

  if (argc != 2) {
    puts("Usage: boolean2 file.tri");
    return 1;
  }

  const char *filename = argv[1];

  meshio::tri::Tri_Mesh_ASCII_file_reader tri_file_reader(filename);
  BVH::BVH bvh;

  {
    meshio::stl::BinaryFileWriter stl_file_writer("tri_as_stl.stl");
    while (auto t = tri_file_reader.read_next_triangle()) {
      const auto &t_value = t.value();
      bvh.triangles.emplace_back(t_value[0], t_value[1], t_value[2]);
      stl_file_writer.write_triangle(t_value[0], t_value[1], t_value[2], {});
    }
  }

  std::cout << "Number of triangles = " << bvh.triangles.size() << std::endl;

  Timer timer;
  bvh.update_tree();
  timer.tock("Building BVH");

  std::cout << "Number of leaf nodes = " << bvh.count_leaf_nodes() << std::endl;

  BVH::LeafInfo biggest_leaf = bvh.find_biggest_leaf();
  std::cout << "Largest leaf = " << biggest_leaf.num_tris << " triangles" << std::endl;

  {
    // Write the largest leaf to an STL file
    meshio::stl::BinaryFileWriter binary_stl("out.stl");

    const BVH::Node &node = bvh.nodes[biggest_leaf.index];
    if (not node.is_leaf()) {
      throw;
    }
    for (size_t i = node.first_primitive_index;
         i < (node.first_primitive_index + node.number_of_primitives); i++) {
      const BVH::Triangle &triangle = bvh.triangles[i];
      binary_stl.write_triangle(triangle.verts[0], triangle.verts[1], triangle.verts[2], {});
    }
  }

  std::vector<Intersection_Point> intersection_points;

  CALLGRIND_START_INSTRUMENTATION;
  CALLGRIND_TOGGLE_COLLECT;

  timer.tick();
  for (const auto &t : bvh.triangles) {
    BVH::Segment3D s1{t.verts[0], t.verts[1]};
    BVH::Segment3D s2{t.verts[1], t.verts[2]};
    BVH::Segment3D s3{t.verts[2], t.verts[0]};

    intersect(bvh, s1, intersection_points);
    intersect(bvh, s2, intersection_points);
    intersect(bvh, s3, intersection_points);

    for (const auto &v : t.verts) {
      if (not BVH::contains_point(bvh, v)) {
        puts("Failed");
      }
    }
  }
  timer.tock();

  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_STOP_INSTRUMENTATION;

  std::cout << "Number of intersection points = " << intersection_points.size() << std::endl;

  // TODO: re-triangulate surface
  // TODO: export

  return 0;
}