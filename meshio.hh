#pragma once

#include <array>
#include <cstdio>
#include <fstream>
#include <vector>

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

struct File {
  uint8_t header[80];
  uint32_t number_of_triangles;
  std::vector<BinaryTriangle> triangles;
};

// TODO: reduce code duplication
std::vector<BinaryTriangle>
read_binary_stl_tris_unchecked(const char *filename) {
  FILE *file = fopen(filename, "rb");
  uint8_t header[80];
  uint32_t tris_num;
  float custom_normal[3];
  float vert[3];
  uint16_t attribute_byte_count;
  std::vector<BinaryTriangle> output;

  fread(header, sizeof(header), 1, file);
  fread(&tris_num, sizeof(tris_num), 1, file);
  output.resize(tris_num * 3);

  fread(&output[0], tris_num * sizeof(BinaryTriangle), 1, file);
  fclose(file);

  return output;
}

std::vector<Vec3> read_binary_stl_vertices_unchecked(const char *filename) {
  FILE *file = fopen(filename, "rb");
  uint8_t header[80];
  uint32_t tris_num;
  float custom_normal[3];
  float vert[3];
  uint16_t attribute_byte_count;
  std::vector<Vec3> output;

  fread(header, sizeof(header), 1, file);
  fread(&tris_num, sizeof(tris_num), 1, file);
  output.reserve(tris_num * 3);

  for (int i = 0; i < tris_num; i++) {
    fread(custom_normal, sizeof(custom_normal), 1, file);
    for (int j = 0; j < 3; j++) {
      fread(vert, sizeof(vert), 1, file);
      output.emplace_back(vert[0], vert[1], vert[2]);
    }
    fread(&attribute_byte_count, sizeof(attribute_byte_count), 1, file);
  }
  fclose(file);

  return output;
}

std::vector<std::pair<size_t, Vec3>>
read_binary_stl_index_vertex_pairs_unchecked(const char *filename) {
  FILE *file = fopen(filename, "rb");
  uint8_t header[80];
  uint32_t tris_num;
  float custom_normal[3];
  float vert[3];
  uint16_t attribute_byte_count;
  std::vector<std::pair<size_t, Vec3>> output;

  fread(header, sizeof(header), 1, file);
  fread(&tris_num, sizeof(tris_num), 1, file);
  output.reserve(tris_num * 3);

  for (int i = 0; i < tris_num; i++) {
    fread(custom_normal, sizeof(custom_normal), 1, file);
    for (int j = 0; j < 3; j++) {
      fread(vert, sizeof(vert), 1, file);
      output.emplace_back(i * 3 + j, Vec3{vert[0], vert[1], vert[2]});
    }
    fread(&attribute_byte_count, sizeof(attribute_byte_count), 1, file);
  }
  fclose(file);

  return output;
}

} // namespace meshio::stl
