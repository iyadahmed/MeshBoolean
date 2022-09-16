#pragma once

#include "../vec3.hh"

namespace meshio::stl {

#pragma pack(push, 1)

struct BinaryTriangle {
  Vec3 custom_normal;
  Vec3 vertices[3];
  uint16_t attribute_byte_count = 0;
};

#pragma pack(pop)
} // namespace meshio::stl