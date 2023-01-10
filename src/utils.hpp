#ifndef BTRACK_UTILS_HPP
#define BTRACK_UTILS_HPP

#include <numbers>

template <typename T> void normalize_array(std::vector<T> &array) {
  double sum = std::reduce(array.begin(), array.end());

  if (sum > 0) {
    for (auto &v : array) {
      v = v / sum;
    }
  }
};

static double calculate_mean_of_array(std::vector<double>::const_iterator begin,
                                      std::vector<double>::const_iterator end) {
  if (begin == end) {
    return 0;
  }

  auto const count = static_cast<float>(end - begin);
  return std::reduce(begin, end) / count;
}

static double princarg(double phase_val) {
  // if phase value is less than or equal to -pi then add 2*pi
  while (phase_val <= (-std::numbers::pi)) {
    phase_val = phase_val + (2 * std::numbers::pi);
  }

  // if phase value is larger than pi, then subtract 2*pi
  while (phase_val > std::numbers::pi) {
    phase_val = phase_val - (2 * std::numbers::pi);
  }

  return phase_val;
}

#endif // BTRACK_UTILS_HPP