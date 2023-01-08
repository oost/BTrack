#ifndef DETECTION_FUNCTION__ENERGY_ENVELOPE_HPP
#define DETECTION_FUNCTION__ENERGY_ENVELOPE_HPP

#include <algorithm>
#include <numeric>
#include <span>

#include "transformers/detection_function.hpp"

namespace transformers {

class EnergyEnvelope : public DetectionFunction<double> {
public:
protected:
  void process() override {
    (*outputBuffer_)[0] = std::inner_product(inputBuffer_->data().begin(),
                                             inputBuffer_->data().end(),
                                             inputBuffer_->data().begin(), 0);
  }
};
} // namespace transformers

#endif // DETECTION_FUNCTION__ENERGY_ENVELOPE_HPP