#pragma once

#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "../non_copyable.hh"
#include "../vec3.hh"
#include "fread_e.hh"
#include "stl_binary_triangle.hh"

namespace meshio::stl {
class BinaryFileWriter : NonCopyable {
private:
  FILE *file;
  uint8_t header[80]{};
  uint32_t number_of_triangles = 0;

public:
  explicit BinaryFileWriter(const char *filename) {
    file = fopen(filename, "wb");
    if (file == nullptr) {
      throw std::runtime_error("Error opening file");
    }
    fwrite(header, sizeof(header), 1, file);
  }

  ~BinaryFileWriter() {
    fwrite(&number_of_triangles, sizeof(number_of_triangles), 1, file);
    fclose(file);
  }

  void write_triangle(const Vec3 &a, const Vec3 &b, const Vec3 &c,
                      const Vec3 &custom_normal) {
    BinaryTriangle t{custom_normal, {a, b, c}, 0};
    fwrite(&t, sizeof(BinaryTriangle), 1, file);
  }
};
} // namespace meshio::stl