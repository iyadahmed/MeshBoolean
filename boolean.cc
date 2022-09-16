#include <iostream>

#include "BVH/bvh.hh"
#include "BVH/query_point.hh"
#include "BVH/segment_bvh_intersection.hh"
#include "meshio/stl_binary_reader.hh"
#include "meshio/stl_binary_writer.hh"
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
  BVH::BVH bvh;

  size_t tris_num = stl_file_reader.get_reported_number_of_triangles();
  for (size_t i = 0; i < tris_num; i++) {
    auto t = stl_file_reader.read_next_triangle();
    bvh.triangles.push_back({t.vertices[0], t.vertices[1], t.vertices[2]});
  }
  std::cout << "Number of triangles = " << bvh.triangles.size() << std::endl;

  Timer timer;
  bvh.update_tree();
  timer.tock("Building BVH");

  std::cout << "Number of leaf nodes = " << bvh.count_leaf_nodes() << std::endl;

  BVH::LeafInfo biggest_leaf = bvh.find_biggest_leaf();
  std::cout << "Largest leaf = " << biggest_leaf.num_tris << " triangles"
            << std::endl;

  {
    meshio::stl::BinaryFileWriter binary_stl("out.stl");

    const BVH::Node &node = bvh.nodes[biggest_leaf.index];
    if (not node.is_leaf()) {
      throw;
    }
    for (size_t i = node.first_primitive_index; i <= node.last_primitive_index;
         i++) {
      const BVH::Triangle &triangle = bvh.triangles[i];
      binary_stl.write_triangle(triangle.verts[0], triangle.verts[1],
                                triangle.verts[2], {});
    }
  }

  //  std::vector<Intersection_Point> intersection_points;

  CALLGRIND_START_INSTRUMENTATION;
  CALLGRIND_TOGGLE_COLLECT;

  timer.tick();
  for (const auto &t : bvh.triangles) {
    // FIXME: slow segment intersection
    //    BVH::Segment3D s1{t.verts[0], t.verts[1]};
    //    BVH::Segment3D s2{t.verts[1], t.verts[2]};
    //    BVH::Segment3D s3{t.verts[2], t.verts[0]};
    //
    //    intersect(bvh, s1, intersection_points);
    //    intersect(bvh, s2, intersection_points);
    //    intersect(bvh, s3, intersection_points);

    for (const auto &v : t.verts) {
      if (not BVH::contains_point(bvh, v)) {
        puts("Failed");
      }
    }
  }
  timer.tock();

  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_STOP_INSTRUMENTATION;

  //  std::cout << "Number of intersection points = " <<
  //  intersection_points.size()
  //            << std::endl;

  // TODO: re-triangulate surface
  // TODO: export

  return 0;
}