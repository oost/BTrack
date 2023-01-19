#ifndef BTRACK__TRANSFORMERS__SHIFT_HPP
#define BTRACK__TRANSFORMERS__SHIFT_HPP

#include <algorithm>
#include <numeric>
#include <span>
#include <vector>

#include "transformers/transformers/all.h"

namespace transformers {

template <typename T>
class ShiftTransformer : public BufferedTransformer<T, T> {
public:
  ShiftTransformer(std::size_t frameSize)
      : BufferedTransformer<T, T>(frameSize) {}

protected:
  void process() override {

    if (this->input_buffer_->size() > this->output_buffer_->size()) {
      throw std::range_error("Frame size should be even");
    }

    int lag = this->input_buffer_->size();
    int numSamplesToKeep = this->output_buffer_->size() - lag;

    // auto samples_to_keep = this->output_buffer_->data() |
    // ranges::copy(this->output_buffer_->data().begin() + numSamplesToKeep,
    //              this->output_buffer_->data().end(),
    //              this->output_buffer_->data().begin());
    for (int i = 0; i < numSamplesToKeep; i++) {
      (*this->output_buffer_)[i] = (*this->output_buffer_)[i + lag];
    }

    for (int i = 0; i < lag; i++) {
      (*this->output_buffer_)[numSamplesToKeep + i] = (*this->input_buffer_)[i];
    }
  };
};

} // namespace transformers

#endif // BTRACK__TRANSFORMERS__SHIFT_HPP