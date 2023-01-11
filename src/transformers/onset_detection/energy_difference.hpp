#ifndef DETECTION_FUNCTION__ENERGY_DIFFERENCE_HPP
#define DETECTION_FUNCTION__ENERGY_DIFFERENCE_HPP

#include <algorithm>
#include <numeric>
#include <span>

#include "transformers/detection_function.hpp"

namespace transformers {

class EnergyDifference : public DetectionFunction<double> {
public:
protected:
  void process() override {
    double sum;
    double sample;

    sum = std::inner_product(input_buffer_->data().begin(),
                             input_buffer_->data().end(),
                             input_buffer_->data().begin(), 0);

    sample =
        sum - prev_energy_sum_; // sample is first order difference in energy

    prev_energy_sum_ = sum; // store energy value for next calculation

    output_buffer_->value() = (sample > 0) ? sample : 0; // return difference
  }

  double prev_energy_sum_ = 0;
};

} // namespace transformers

#endif // DETECTION_FUNCTION__ENERGY_DIFFERENCE_HPP