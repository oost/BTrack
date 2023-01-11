#ifndef DETECTION_FUNCTION__COMPLEX_SPECTRAL_DIFFERENCE_HWR_HPP
#define DETECTION_FUNCTION__COMPLEX_SPECTRAL_DIFFERENCE_HWR_HPP

#include <algorithm>
#include <numeric>
#include <span>
#include <vector>

#include "transformers/detection_function.hpp"

namespace transformers {

class ComplexSpectralDifferenceHWR
    : public DetectionFunction<std::complex<double>> {
public:
  ComplexSpectralDifferenceHWR(std::size_t input_size)
      : DetectionFunction(), mag_spec_(input_size, 0.0),
        prev_mag_spec_(input_size, 0.0), phase_(input_size, 0.0),
        prev_phase_(input_size, 0.0), prev_phase2_(input_size, 0.0) {}

protected:
  void process() override {
    double phase_deviation;
    double sum;
    double magnitude_difference;
    double csd;

    sum = 0; // initialise sum to zero

    // compute phase values from fft output and sum deviations
    for (int i = 0; i < input_buffer_->size(); i++) {
      // calculate phase value
      // phase[i] = atan2(fft_operator_->output()[i].imag(),
      //                  fft_operator_->output()[i].real());
      phase_[i] = std::arg((*input_buffer_)[i]);
      // calculate magnitude value
      mag_spec_[i] = std::abs((*input_buffer_)[i]);

      // phase deviation
      phase_deviation = phase_[i] - (2 * prev_phase_[i]) + prev_phase2_[i];

      // calculate magnitude difference (real part of Euclidean distance between
      // complex frames)
      magnitude_difference = mag_spec_[i] - prev_mag_spec_[i];

      // if we have a positive change in magnitude, then include in sum,
      // otherwise ignore (half-wave rectification)
      if (magnitude_difference > 0) {
        // calculate complex spectral difference for the current spectral bin
        csd = sqrt(pow(mag_spec_[i], 2) + pow(prev_mag_spec_[i], 2) -
                   2 * mag_spec_[i] * prev_mag_spec_[i] * cos(phase_deviation));

        // add to sum
        sum = sum + csd;
      }

      // store values for next calculation
      prev_phase2_[i] = prev_phase_[i];
      prev_phase_[i] = phase_[i];
      prev_mag_spec_[i] = mag_spec_[i];
    }

    output_buffer_->value() = sum;
  }

  std::vector<double> mag_spec_;
  std::vector<double> prev_mag_spec_;

  std::vector<double> phase_;
  std::vector<double> prev_phase_;
  std::vector<double> prev_phase2_;
};

} // namespace transformers

#endif // DETECTION_FUNCTION__COMPLEX_SPECTRAL_DIFFERENCE_HWR_HPP