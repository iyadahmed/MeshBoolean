#pragma once

#include <array>
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

#pragma pack(push, 1)

struct STLBinaryTriangle {
  Vec3 custom_normal;
  std::array<Vec3, 3> verts;
  uint16_t attribute_byte_count;
};

#pragma pack(pop)

std::vector<STLBinaryTriangle>
read_binary_stl_unchecked(std::string const &filename) {
  std::ifstream file(filename);
  uint8_t header[80];
  uint32_t tris_num;

  file.read((char *)header, 80);
  file.read((char *)&tris_num, sizeof(uint32_t));
  std::vector<STLBinaryTriangle> result(tris_num);
  file.read((char *)result.data(), result.size() * sizeof(STLBinaryTriangle));
  return result;
}
