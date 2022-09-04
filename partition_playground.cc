#include <functional>
#include <iostream>
#include <random>
#include <vector>

#include "common.hh"

int main() {

  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(1, 100000);

  std::vector<int> points;

  for (int i = 0; i < 100000; i++) {
    points.push_back(distribution(generator));
  }

  int s = 1000;

  size_t i = 0;
  size_t k = points.size() - 1;

  std::cout << "i = " << i << " k = " << k << std::endl;
  while (i < k) {
    if (points[i] < s) {
      i++;
    } else {
      std::swap(points[i], points[k]);
      tassert(k != 0);
      k--;
    }
  }

  for (auto &point : points) {
    std::cout << point << " " << std::endl;
  }
}