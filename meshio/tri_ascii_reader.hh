#pragma once

#include <array>
#include <cerrno>
#include <cstdlib>
#include <optional>
#include <stdexcept>

#include "../non_copyable.hh"
#include "../vec3.hh"

namespace meshio::tri {
using Triangle = std::array<Vec3, 3>;

class Tri_Mesh_ASCII_file_reader : NonCopyable {
private:
  static constexpr int MAX_LINE_LENGTH = 100;

  FILE *file;
  uint8_t header[80]{};
  uint32_t number_of_triangles = 0;

public:
  explicit Tri_Mesh_ASCII_file_reader(const char *filename) {
    file = fopen(filename, "r");
    if (file == nullptr) {
      throw std::runtime_error("Error opening file");
    }
  }

  ~Tri_Mesh_ASCII_file_reader() { fclose(file); }

  std::optional<Triangle> read_next_triangle() {
    char line[MAX_LINE_LENGTH + 1];
    if (fgets(line, MAX_LINE_LENGTH, file) == nullptr) {
      if (ferror(file)) {
        throw std::runtime_error("Error reading file");
      } else if (feof(file)) {
        return std::nullopt;
      }
    }
    Triangle t;
    //    if (fscanf(file, "%f %f %f %f %f %f %f %f %f\n", &t[0][0], &t[0][1],
    //               &t[0][2], &t[1][0], &t[1][1], &t[1][2], &t[2][0], &t[2][1],
    //               &t[2][2]) != 9) {
    //      return std::nullopt;
    //    }
    line[MAX_LINE_LENGTH] = '\0';
    char *c = line;
    char *end = c;
    float *values = reinterpret_cast<float *>(&t[0][0]);
    for (int i = 0; i < 9; i++) {
      errno = 0;
      values[i] = strtof(c, &end);
      if (c == end) {
        throw std::runtime_error("Failed to parse float value");
      }
      if (errno == ERANGE) {
        throw std::out_of_range("Parsed float value is out of range");
      }
      c = end;
    }
    return t;
  }
};
} // namespace meshio::tri