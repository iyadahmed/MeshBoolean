#pragma once

#include <cstddef>
#include <cstdio>
#include <stdexcept>

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