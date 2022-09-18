#pragma once

#include <array>

namespace meshio::stl {

#pragma pack(push, 1)

struct BinaryTriangle {
  std::array<float, 3> custom_normal;
  std::array<float, 3> vertices[3];
  uint16_t attribute_byte_count = 0;
};

#pragma pack(pop)
} // namespace meshio::stl