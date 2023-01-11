#ifndef DETECTION_FUNCTION__SPECTRAL_DIFFERENCE_HWR_HPP
#define DETECTION_FUNCTION__SPECTRAL_DIFFERENCE_HWR_HPP

#include <algorithm>
#include <complex>
#include <numeric>
#include <span>
#include <vector>

#include "transformers/detection_function.hpp"

namespace transformers {

class SpectralDifferenceHWR : public DetectionFunction<std::complex<double>> {
public:
  SpectralDifferenceHWR(std::size_t input_size)
      : DetectionFunction(), mag_spec_(input_size, 0.0),
        prev_mag_spec_(input_size, 0.0) {}

protected:
  void process() override {
    double diff;
    double sum;

    // std::transform(
    //     input.begin(), input.end(), mag_spec_.begin(),
    //     [](std::complex<double> a) -> double { return std::abs(a); });
    std::transform(input_buffer_->data().begin(), input_buffer_->data().end(),
                   mag_spec_.begin(), std::abs<double>);
    // // compute first (N/2)+1 mag values
    // for (int i = 0; i < (frameSize_ / 2) + 1; i++) {
    //   mag_spec_[i] = std::abs(fft_operator_->output()[i]);
    // }
    // // mag spec symmetric above (N/2)+1 so copy previous values
    // for (int i = (frameSize_ / 2) + 1; i < frameSize_; i++) {
    //   mag_spec_[i] = mag_spec_[frameSize_ - i];
    // }

    sum = 0; // initialise sum to zero

    for (int i = 0; i < input_buffer_->size(); i++) {
      // calculate difference
      diff = mag_spec_[i] - prev_mag_spec_[i];

      // only add up positive differences
      if (diff > 0) {
        // add difference to sum
        sum = sum + diff;
      }

      // store magnitude spectrum bin for next detection function sample
      // calculation
      prev_mag_spec_[i] = mag_spec_[i];
    }

    output_buffer_->value() = sum;
  }

  std::vector<double> mag_spec_;
  std::vector<double> prev_mag_spec_;
};
} // namespace transformers

#endif // DETECTION_FUNCTION__SPECTRAL_DIFFERENCE_HWR_HPP