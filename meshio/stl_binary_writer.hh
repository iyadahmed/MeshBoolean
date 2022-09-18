#pragma once

#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "../non_copyable.hh"
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
    // Write number of triangles, so we can write over it later
    fwrite(&number_of_triangles, sizeof(number_of_triangles), 1, file);
  }

  ~BinaryFileWriter() {
    fseek(file, sizeof(header), SEEK_SET);
    fwrite(&number_of_triangles, sizeof(number_of_triangles), 1, file);
    fclose(file);
  }

  void write_triangle(const Vec3 &a, const Vec3 &b, const Vec3 &c,
                      const Vec3 &custom_normal) {
    BinaryTriangle t{{custom_normal.x, custom_normal.y, custom_normal.z},
                     {{a.x, a.y, a.z}, {b.x, b.y, b.z}, {c.x, c.y, c.z}},
                     0};
    size_t n = fwrite(&t, sizeof(BinaryTriangle), 1, file);
    if (n == 1) {
      number_of_triangles++;
    } else {
      throw std::runtime_error("Error writing triangle");
    }
  }
};
} // namespace meshio::stl