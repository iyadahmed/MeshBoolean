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

  static float read_float(char **string) {
    errno = 0;
    char *end = *string;
    float res = strtof(*string, &end);
    if (*string == end) {
      throw std::runtime_error("Failed to parse float value");
    }
    if (errno == ERANGE) {
      throw std::out_of_range("Parsed float value is out of range");
    }
    *string = end;
    return res;
  }

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
    line[MAX_LINE_LENGTH] = '\0';
    char *str = line;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        t[i][j] = read_float(&str);
      }
    }
    return t;
  }
};
} // namespace meshio::tri