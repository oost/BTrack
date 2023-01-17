#ifndef DETECTION_FUNCTION__PHASE_DEVIATION_HPP
#define DETECTION_FUNCTION__PHASE_DEVIATION_HPP

#include <algorithm>
#include <numeric>
#include <span>
#include <vector>

#include "../utils.h"
#include "transformers/detection_function.hpp"

namespace transformers {

class PhaseDeviation : public DetectionFunction<std::complex<double>> {
public:
  PhaseDeviation(std::size_t input_size)
      : DetectionFunction(), mag_spec_(input_size, 0.0),
        prev_mag_spec_(input_size, 0.0), phase_(input_size, 0.0),
        prev_phase_(input_size, 0.0), prev_phase2_(input_size, 0.0) {}

protected:
  void process() override {

    double dev, pdev;
    double sum;

    sum = 0; // initialise sum to zero

    // compute phase values from fft output and sum deviations
    for (int i = 0; i < input_buffer_->size(); i++) {
      // calculate phase value
      // phase[i] = atan2(fft_operator_->output()[i].imag(),
      //                  fft_operator_->output()[i].real());
      phase_[i] = std::arg((*input_buffer_)[i]);
      // calculate magnitude value
      mag_spec_[i] = std::abs((*input_buffer_)[i]);

      // if bin is not just a low energy bin then examine phase deviation
      if (mag_spec_[i] > 0.1) {
        dev = phase_[i] - (2 * prev_phase_[i]) +
              prev_phase2_[i]; // phase deviation
        pdev = princarg(dev);  // wrap into [-pi,pi] range

        // make all values positive
        if (pdev < 0) {
          pdev = pdev * -1;
        }

        // add to sum
        sum = sum + pdev;
      }

      // store values for next calculation
      prev_phase2_[i] = prev_phase_[i];
      prev_phase_[i] = phase_[i];
    }

    output_buffer_->set_value(sum);
  }

  std::vector<double> mag_spec_;
  std::vector<double> prev_mag_spec_;

  std::vector<double> phase_;
  std::vector<double> prev_phase_;
  std::vector<double> prev_phase2_;
};
} // namespace transformers

#endif // DETECTION_FUNCTION__PHASE_DEVIATION_HPP