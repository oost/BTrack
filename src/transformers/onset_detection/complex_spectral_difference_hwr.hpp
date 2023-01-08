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
  ComplexSpectralDifferenceHWR(std::size_t inputSize)
      : DetectionFunction(), magSpec_(inputSize, 0.0),
        prevMagSpec_(inputSize, 0.0), phase_(inputSize, 0.0),
        prevPhase_(inputSize, 0.0), prevPhase2_(inputSize, 0.0) {}

protected:
  void process() override {
    double phaseDeviation;
    double sum;
    double magnitudeDifference;
    double csd;

    sum = 0; // initialise sum to zero

    // compute phase values from fft output and sum deviations
    for (int i = 0; i < inputBuffer_->size(); i++) {
      // calculate phase value
      // phase[i] = atan2(fft_operator_->output()[i].imag(),
      //                  fft_operator_->output()[i].real());
      phase_[i] = std::arg((*inputBuffer_)[i]);
      // calculate magnitude value
      magSpec_[i] = std::abs((*inputBuffer_)[i]);

      // phase deviation
      phaseDeviation = phase_[i] - (2 * prevPhase_[i]) + prevPhase2_[i];

      // calculate magnitude difference (real part of Euclidean distance between
      // complex frames)
      magnitudeDifference = magSpec_[i] - prevMagSpec_[i];

      // if we have a positive change in magnitude, then include in sum,
      // otherwise ignore (half-wave rectification)
      if (magnitudeDifference > 0) {
        // calculate complex spectral difference for the current spectral bin
        csd = sqrt(pow(magSpec_[i], 2) + pow(prevMagSpec_[i], 2) -
                   2 * magSpec_[i] * prevMagSpec_[i] * cos(phaseDeviation));

        // add to sum
        sum = sum + csd;
      }

      // store values for next calculation
      prevPhase2_[i] = prevPhase_[i];
      prevPhase_[i] = phase_[i];
      prevMagSpec_[i] = magSpec_[i];
    }

    (*outputBuffer_)[0] = sum;
  }

  std::vector<double> magSpec_;
  std::vector<double> prevMagSpec_;

  std::vector<double> phase_;
  std::vector<double> prevPhase_;
  std::vector<double> prevPhase2_;
};

} // namespace transformers

#endif // DETECTION_FUNCTION__COMPLEX_SPECTRAL_DIFFERENCE_HWR_HPP