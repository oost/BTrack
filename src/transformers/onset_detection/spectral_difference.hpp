#ifndef DETECTION_FUNCTION__SPECTRAL_DIFFERENCE_HPP
#define DETECTION_FUNCTION__SPECTRAL_DIFFERENCE_HPP

#include <algorithm>
#include <complex>
#include <numeric>
#include <span>
#include <vector>

#include "transformers/detection_function.hpp"

namespace transformers {

class SpectralDifference : public DetectionFunction<std::complex<double>> {
public:
  SpectralDifference(std::size_t inputSize)
      : DetectionFunction(), magSpec_(inputSize, 0.0),
        prevMagSpec_(inputSize, 0.0) {}

protected:
  void process() override {
    double diff;
    double sum;

    std::transform(inputBuffer_->data().begin(), inputBuffer_->data().end(),
                   magSpec_.begin(), std::abs<double>);

    // // compute first (N/2)+1 mag values
    // for (int i = 0; i < (frameSize_ / 2) + 1; i++) {
    //   magSpec_[i] = std::abs(fft_operator_->output()[i]);
    // }
    // // mag spec symmetric above (N/2)+1 so copy previous values
    // for (int i = (frameSize_ / 2) + 1; i < frameSize_; i++) {
    //   magSpec_[i] = magSpec_[frameSize_ - i];
    // }

    sum = 0; // initialise sum to zero

    for (int i = 0; i < inputBuffer_->size(); i++) {
      // calculate difference
      diff = magSpec_[i] - prevMagSpec_[i];

      // ensure all difference values are positive
      if (diff < 0) {
        diff = diff * -1;
      }

      // add difference to sum
      sum = sum + diff;

      // store magnitude spectrum bin for next detection function sample
      // calculation
      prevMagSpec_[i] = magSpec_[i];
    }

    (*outputBuffer_)[0] = sum;
  }

  std::vector<double> magSpec_;
  std::vector<double> prevMagSpec_;
};
} // namespace transformers

#endif // DETECTION_FUNCTION__SPECTRAL_DIFFERENCE_HPP