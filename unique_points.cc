#include <algorithm>
#include <iostream>
#include <tuple>
#include <vector>

#include "meshio/stl_binary_reader.hh"
#include "timers.hh"

using namespace meshio::stl;

using SortableVertex = std::tuple<float, float, float, size_t>;

size_t get_triangle_index(const SortableVertex &v) { return std::get<3>(v); }

Vec3 as_vec3(const SortableVertex &v) {
  return {std::get<0>(v), std::get<1>(v), std::get<2>(v)};
}

int main(int argc, const char *argv[]) {

  if (argc != 2) {
    puts("Usage: unique file.stl");
    return 1;
  }

  const char *filename = argv[1];

  BinaryFileReader binary_stl(filename);

  size_t n = binary_stl.get_reported_number_of_triangles();

  std::vector<SortableVertex> verts;

  for (size_t i = 0; i < n; i++) {
    auto t = binary_stl.read_next_triangle();
    for (auto &v : t.vertices) {
      verts.emplace_back(v.x, v.y, v.z, i);
    }
  }

  std::cout << "Number of non-unique vertices = " << verts.size() << std::endl;

  std::sort(verts.begin(), verts.end());

  std::vector<size_t> unique_vertex_indices_map;
  unique_vertex_indices_map.resize(verts.size());
  size_t first_index = get_triangle_index(verts[0]);
  unique_vertex_indices_map[first_index] = first_index;
  size_t unique_verts_num = 1;

  for (size_t i = 1; i < verts.size(); i++) {
    Vec3 v1 = as_vec3(verts[i]);
    Vec3 v2 = as_vec3(verts[i - 1]);
    size_t v1_i = get_triangle_index(verts[i]);
    size_t v2_i = get_triangle_index(verts[i - 1]);

    if (distance(v1, v2) < 0.0000001f) {
      unique_vertex_indices_map[v1_i] = unique_vertex_indices_map[v2_i];
    } else {
      unique_verts_num++;
    }
  }

  std::cout << "Number of unique vertices = " << unique_verts_num << std::endl;

  return 0;
}