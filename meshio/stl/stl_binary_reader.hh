#pragma once

#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "../../non_copyable.hh"
#include "../fread_e.hh"
#include "stl_binary_triangle.hh"

namespace meshio::stl {
class BinaryFileReader : NonCopyable {
private:
  FILE *file;
  uint8_t header[80]{};
  uint32_t number_of_triangles = 0;

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

  size_t get_reported_number_of_triangles() const { return number_of_triangles; }

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
} // namespace meshio::stl