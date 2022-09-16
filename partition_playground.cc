#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <random>
#include <vector>

#include "common.hh"

int main() {
  //
  //  std::default_random_engine generator;
  //  std::uniform_int_distribution<int> distribution(1, 100000);
  //
  //  std::vector<int> points;
  //
  //  for (int i = 0; i < 100000; i++) {
  //    points.push_back(distribution(generator));
  //  }
  //
  //  int s = 1000;
  //
  //  size_t i = 0;
  //  size_t k = points.size() - 1;
  //
  //  std::cout << "i = " << i << " k = " << k << std::endl;
  //  while (i < k) {
  //    if (points[i] < s) {
  //      i++;
  //    } else {
  //      std::swap(points[i], points[k]);
  //      assert(k != 0);
  //      k--;
  //    }
  //  }
  //
  //  for (auto &point : points) {
  //    std::cout << point << " " << std::endl;
  //  }
  //
  //  std::vector<int> foo(10);
  //  std::iota(foo.begin(), foo.end(), 0);
  //  auto bar = std::partition(foo.begin(), foo.end(), [](int i) { return i <
  //  11; }); assert(bar != foo.end());
  //
  //  std::vector<int> numbers(100000000);
  //  std::iota(numbers.begin(), numbers.end(), 0);
  //
  //  std::vector<int> p1, p2;
  //  p1.reserve(numbers.size());
  //  p2.reserve(numbers.size());
  //
  //  for (size_t i = 0; i < numbers.size(); i++) {
  //  }

  std::vector<int> foo(10);
  std::iota(foo.begin(), foo.end(), 1);

  size_t right_partition_count = 0;
  size_t left_partition_start = foo.size();

  while (right_partition_count < left_partition_start) {
    if (foo[right_partition_count] > 10) {
      right_partition_count++;
    } else {
      left_partition_start--;
      std::swap(foo[right_partition_count], foo[left_partition_start]);
    }
  }

  std::cout << right_partition_count << std::endl;
  std::cout << left_partition_start << std::endl;
}