#include <algorithm>
#include <iostream>
#include <utility> // for std::pair

#include "meshio.hh"

using namespace meshio::stl;

bool cmp(const std::pair<size_t, Vec3> &ap, const std::pair<size_t, Vec3> &bp) {
  const Vec3 &a = ap.second;
  const Vec3 &b = bp.second;
  // Strict weak ordering from https://stackoverflow.com/a/22155743/8094047
  // NOTE: beware of this https://stackoverflow.com/a/6978325/8094047
  if (a[0] == b[0]) {
    if (a[1] == b[1]) {
      return a[2] < b[2];
    } else {
      return a[1] < b[1];
    }
  } else {
    return a[0] < b[0];
  }
}

int main(int argc, const char *argv[]) {

  if (argc != 2) {
    puts("Usage: unique file.stl");
    return 1;
  }

  const char *filename = argv[1];

  auto verts = read_binary_stl_index_vertex_pairs_unchecked(filename);
  std::cout << "Number of non-unique vertices = " << verts.size() << std::endl;

  std::sort(verts.begin(), verts.end(), cmp);

  std::vector<size_t> unique_vertex_indices_map;
  unique_vertex_indices_map.resize(verts.size());
  unique_vertex_indices_map[verts[0].first] = verts[0].first;
  size_t unique_verts_num = 1;

  for (size_t i = 1; i < verts.size(); i++) {
    if (distance(verts[i].second, verts[i - 1].second) < 0.0000001f) {
      unique_vertex_indices_map[verts[i].first] =
          unique_vertex_indices_map[verts[i - 1].first];
    } else {
      unique_verts_num++;
    }
  }

  std::cout << "Number of non-unique vertices = " << unique_verts_num
            << std::endl;

  return 0;
}