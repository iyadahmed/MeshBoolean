#include <functional>
#include <iostream>
#include <random>
#include <vector>

int main() {

  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(1, 100000);
  auto get_int = std::bind(distribution, generator);

  std::vector<int> points;

  for (int i = 0; i < 100000; i++) {
    points.push_back(get_int());
  }

  int s = 1000;

  size_t i = 0;
  size_t k = points.size() - 1;

  std::cout << "i = " << i << " k = " << k << std::endl;
  while (i <= k) {
    if (points[i] < s) {
      i++;
    } else {
      std::swap(points[i], points[k]);
      k--;
    }
  }

  for (auto& point : points) {
    std::cout << point << " " << std::endl;
  }

//  std::cout << i << " " << k << std::endl;
}