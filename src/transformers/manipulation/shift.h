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
    int samplesToKeep = this->output_buffer_->size() - lag;
    for (int i = 0; i < samplesToKeep; i++) {
      (*this->output_buffer_)[i] = (*this->output_buffer_)[i + lag];
    }

    for (int i = 0; i < lag; i++) {
      (*this->output_buffer_)[samplesToKeep + i] = (*this->input_buffer_)[i];
    }
  };
};

} // namespace transformers

#endif // BTRACK__TRANSFORMERS__SHIFT_HPP