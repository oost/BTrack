#ifndef BTRACK__HIGH_FREQUENCY_SPECTRAL_DIFFERENCE_HWR_HPP
#define BTRACK__HIGH_FREQUENCY_SPECTRAL_DIFFERENCE_HWR_HPP

#include <algorithm>
#include <numeric>
#include <span>
#include <vector>

#include "transformers/detection_function.hpp"

namespace transformers {

class HighFrequencySpectralDifferenceHWR
    : public DetectionFunction<std::complex<double>> {
public:
  HighFrequencySpectralDifferenceHWR(std::size_t input_size)
      : DetectionFunction(), magSpec_(input_size, 0.0),
        prevMagSpec_(input_size, 0.0) {}

protected:
  void process() {
    double sum;
    double mag_diff;

    sum = 0; // initialise sum to zero

    // compute phase values from fft output and sum deviations
    for (int i = 0; i < input_buffer_->size(); i++) {
      // calculate magnitude value
      magSpec_[i] = std::abs((*input_buffer_)[i]);

      // calculate difference
      mag_diff = magSpec_[i] - prevMagSpec_[i];

      if (mag_diff > 0) {
        sum = sum + (mag_diff * static_cast<double>(i + 1));
      }

      // store values for next calculation
      prevMagSpec_[i] = magSpec_[i];
    }

    (*input_buffer_)[0] = sum;
  }

  std::vector<double> magSpec_;
  std::vector<double> prevMagSpec_;
};
} // namespace transformers

#endif // BTRACK__HIGH_FREQUENCY_SPECTRAL_DIFFERENCE_HWR_HPP