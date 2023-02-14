#ifndef BTRACK__HIGH_FREQUENCY_CONTENT_HPP
#define BTRACK__HIGH_FREQUENCY_CONTENT_HPP

#include <algorithm>
#include <numeric>
#include <span>
#include <vector>

#include "transformers/detection_function.hpp"

namespace btrack::transformers {

class HighFrequencyContent : public DetectionFunction<std::complex<double>> {
public:
  HighFrequencyContent(std::size_t input_size)
      : DetectionFunction(), mag_spec_(input_size, 0.0),
        prev_mag_spec_(input_size, 0.0) {}

protected:
  void process() override {
    double sum = 0; // initialise sum to zero

    // compute phase values from fft output and sum deviations
    for (int i = 0; i < input_buffer_->size(); i++) {
      // calculate magnitude value
      mag_spec_[i] = std::abs((*input_buffer_)[i]);

      sum = sum + (mag_spec_[i] * static_cast<double>(i + 1));

      // store values for next calculation
      prev_mag_spec_[i] = mag_spec_[i];
    }

    output_buffer_->set_value(sum);
  }

  std::vector<double> mag_spec_;
  std::vector<double> prev_mag_spec_;
};
} // namespace btrack::transformers

#endif // BTRACK__HIGH_FREQUENCY_CONTENT_HPP