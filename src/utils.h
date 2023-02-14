#ifndef BTRACK_UTILS_HPP
#define BTRACK_UTILS_HPP

#include <concepts>
#include <numbers>
#include <range/v3/range/concepts.hpp>

namespace btrack {

/** Normalises a given array
 * @param array a pointer to the array we wish to normalise
 * @param N the length of the array
 */
template <typename T>
void normalize_array(T &array)
  requires ranges::bidirectional_range<T>
{
  double sum = std::reduce(array.begin(), array.end());

  if (sum > 0) {
    for (auto &v : array) {
      v = v / sum;
    }
  }
};

/** Calculates the mean of values in an array between index locations
 * [startIndex,endIndex]
 * @param array a pointer to an array that contains the values we wish to find
 * the mean from
 * @param startIndex the start index from which we would like to calculate the
 * mean
 * @param endIndex the final index to which we would like to calculate the
 * mean
 * @returns the mean of the sub-section of the array
 */
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

} // namespace btrack

#endif // BTRACK_UTILS_HPP