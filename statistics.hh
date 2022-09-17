#pragma once

#include <cstddef>

class RunningMean {
private:
  float old_mean_;
  float old_number_of_elements_;

public:
  RunningMean() {
    old_mean_ = 0;
    old_number_of_elements_ = 0;
  }

  void push(float value) {
    old_mean_ *= (old_number_of_elements_ / (old_number_of_elements_ + 1));
    old_number_of_elements_ += 1;
    old_mean_ += value / old_number_of_elements_;
  }

  float const &get_mean() const { return old_mean_; }
};

// TODO: write our own implementation
// Modified from: https://www.johndcook.com/blog/standard_deviation/
template <typename T> class RunningStat {
private:
  size_t m_n;
  T m_oldM, m_newM, m_oldS, m_newS;

public:
  RunningStat() {
    m_n = 0;
    m_oldM = m_newM = m_oldS = m_newS = 0;
  }

  void clear() { m_n = 0; }

  void push(T x) {
    m_n++;

    // See Knuth TAOCP vol 2, 3rd edition, page 232
    if (m_n == 1) {
      m_oldM = m_newM = x;
      m_oldS = 0.0;
    } else {
      m_newM = m_oldM + (x - m_oldM) / m_n;
      m_newS = m_oldS + (x - m_oldM) * (x - m_newM);

      // set up for next iteration
      m_oldM = m_newM;
      m_oldS = m_newS;
    }
  }

  T get_num_values() const { return m_n; }

  T get_mean() const { return (m_n > 0) ? m_newM : 0.0; }

  T get_variance() const { return ((m_n > 1) ? m_newS / (m_n - 1) : 0.0); }

  //  T get_standard_deviation() const { return std::sqrt(get_variance()); }
};