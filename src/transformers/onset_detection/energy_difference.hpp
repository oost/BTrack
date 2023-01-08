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

    sum = std::inner_product(inputBuffer_->data().begin(),
                             inputBuffer_->data().end(),
                             inputBuffer_->data().begin(), 0);

    sample = sum - prevEnergySum_; // sample is first order difference in energy

    prevEnergySum_ = sum; // store energy value for next calculation

    (*outputBuffer_)[0] = (sample > 0) ? sample : 0; // return difference
  }

  double prevEnergySum_ = 0;
};

} // namespace transformers

#endif // DETECTION_FUNCTION__ENERGY_DIFFERENCE_HPP