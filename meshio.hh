#pragma once

#include <array>
#include <cstdio>
#include <fstream>
#include <vector>

#include "common.hh"
#include "non_copyable.hh"
#include "vec3.hh"

/*  Binary STL spec.:
 *   UINT8[80]    – Header                  - 80 bytes
 *   UINT32       – Number of triangles     - 4 bytes
 *   For each triangle                      - 50 bytes:
 *     REAL32[3]   – Normal vector          - 12 bytes
 *     REAL32[3]   – Vertex 1               - 12 bytes
 *     REAL32[3]   – Vertex 2               - 12 bytes
 *     REAL32[3]   – Vertex 3               - 12 bytes
 *     UINT16      – Attribute byte count   -  2 bytes
 */

namespace meshio::stl {

#pragma pack(push, 1)

struct BinaryTriangle {
  Vec3 custom_normal;
  Vec3 vertices[3];
  uint16_t attribute_byte_count = 0;
};

#pragma pack(pop)

class BinaryFileReader : NonCopyable {
private:
  FILE *file;
  uint8_t header[80];
  uint32_t number_of_triangles;

  // fread wrapper with error checking
  static void fread_e(void *output, size_t size, size_t n, FILE *file) {
    size_t num_read_items = fread(output, size, n, file);
    if (num_read_items != n) {
      if (ferror(file)) {
        throw std::runtime_error("Error reading file");
      } else if (feof(file)) {
        throw std::runtime_error("EOF found");
      } else {
        throw std::runtime_error("Unknown file error");
      }
    }
  }

public:
  explicit BinaryFileReader(const char *filename) {
    file = fopen(filename, "rb");
    if (file == nullptr) {
      throw std::runtime_error("Error opening file");
    }
    fread_e(header, sizeof(header), 1, file);
    fread_e(&number_of_triangles, sizeof(number_of_triangles), 1, file);
  }

  ~BinaryFileReader() { fclose(file); }

  size_t get_reported_number_of_triangles() const {
    return number_of_triangles;
  }

  BinaryTriangle read_next_triangle() {
    BinaryTriangle output;
    fread_e(&output, sizeof(BinaryTriangle), 1, file);
    return output;
  }

  std::vector<BinaryTriangle> read_all_triangles() {
    std::vector<BinaryTriangle> output;
    output.resize(number_of_triangles);
    fread_e(&output[0], number_of_triangles * sizeof(BinaryTriangle), 1, file);
    return output;
  }
};

std::vector<std::pair<size_t, Vec3>>
read_binary_stl_index_vertex_pairs_unchecked(const char *filename) {
  BinaryFileReader stl_file_reader(filename);
  std::vector<std::pair<size_t, Vec3>> output;
  for (size_t i = 0; i < stl_file_reader.get_reported_number_of_triangles();
       i++) {
    auto t = stl_file_reader.read_next_triangle();
    for (size_t j = 0; j < 3; j++) {
      const auto &vert = t.vertices[j];
      output.emplace_back(i * 3 + j, Vec3{vert[0], vert[1], vert[2]});
    }
  }
  return output;
}

} // namespace meshio::stl
