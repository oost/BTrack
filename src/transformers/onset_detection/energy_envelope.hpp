#ifndef DETECTION_FUNCTION__ENERGY_ENVELOPE_HPP
#define DETECTION_FUNCTION__ENERGY_ENVELOPE_HPP

#include <algorithm>
#include <numeric>
#include <span>

#include "transformers/detection_function.hpp"

namespace btrack::transformers {

class EnergyEnvelope : public DetectionFunction<double> {
public:
protected:
  void process() override {
    output_buffer_->set_value(std::inner_product(
        input_buffer_->data().begin(), input_buffer_->data().end(),
        input_buffer_->data().begin(), 0));
  }
};
} // namespace btrack::transformers

#endif // DETECTION_FUNCTION__ENERGY_ENVELOPE_HPP