#ifndef DETECTION_FUNCTION__COMPLEX_SPECTRAL_DIFFERENCE_HPP
#define DETECTION_FUNCTION__COMPLEX_SPECTRAL_DIFFERENCE_HPP

#include <algorithm>
#include <numeric>
#include <span>
#include <vector>

#include "transformers/detection_function.hpp"

namespace transformers {

class ComplexSpectralDifference
    : public DetectionFunction<std::complex<double>> {
public:
  ComplexSpectralDifference(std::size_t input_size)
      : DetectionFunction(), magSpec_(input_size, 0.0),
        prevMagSpec_(input_size, 0.0), phase_(input_size, 0.0),
        prevPhase_(input_size, 0.0), prevPhase2_(input_size, 0.0) {}

protected:
  void process() override {
    double phaseDeviation;
    double sum;
    double csd;

    sum = 0; // initialise sum to zero

    // compute phase values from fft output and sum deviations
    for (int i = 0; i < input_buffer_->size(); i++) {
      // calculate phase value
      // phase[i] = atan2(fft_operator_->output()[i].imag(),
      //                  fft_operator_->output()[i].real());
      phase_[i] = std::arg((*this->input_buffer_)[i]);

      // calculate magnitude value
      magSpec_[i] = std::abs((*this->input_buffer_)[i]);

      // phase deviation
      phaseDeviation = phase_[i] - (2 * prevPhase_[i]) + prevPhase2_[i];

      // calculate complex spectral difference for the current spectral bin
      csd = sqrt(pow(magSpec_[i], 2) + pow(prevMagSpec_[i], 2) -
                 2 * magSpec_[i] * prevMagSpec_[i] * cos(phaseDeviation));

      // add to sum
      sum = sum + csd;

      // store values for next calculation
      prevPhase2_[i] = prevPhase_[i];
      prevPhase_[i] = phase_[i];
      prevMagSpec_[i] = magSpec_[i];
    }

    this->output_buffer_->set_value(sum);
  }

  std::vector<double> magSpec_;
  std::vector<double> prevMagSpec_;

  std::vector<double> phase_;
  std::vector<double> prevPhase_;
  std::vector<double> prevPhase2_;
};

} // namespace transformers

#endif // DETECTION_FUNCTION__COMPLEX_SPECTRAL_DIFFERENCE_HPP