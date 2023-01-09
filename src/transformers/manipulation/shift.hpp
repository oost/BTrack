#ifndef BTRACK__TRANSFORMERS__SHIFT_HPP
#define BTRACK__TRANSFORMERS__SHIFT_HPP

#include <algorithm>
#include <numeric>
#include <span>
#include <vector>

#include "transformers/transformer.hpp"

namespace transformers {

template <typename T>
class ShiftTransformer : public BufferedTransformer<T, T> {
public:
  ShiftTransformer(std::size_t frameSize)
      : BufferedTransformer<T, T>(frameSize) {}

protected:
  void process() override {

    if (this->inputBuffer_->size() > this->outputBuffer_->size()) {
      throw std::range_error("Frame size should be even");
    }

    int lag = this->inputBuffer_->size();
    int samplesToKeep = this->outputBuffer_->size() - lag;
    for (int i = 0; i < samplesToKeep; i++) {
      (*this->outputBuffer_)[i] = (*this->outputBuffer_)[i + lag];
    }

    for (int i = 0; i < lag; i++) {
      (*this->outputBuffer_)[samplesToKeep + i] = (*this->inputBuffer_)[i];
    }
  };
};

} // namespace transformers

#endif // BTRACK__TRANSFORMERS__SHIFT_HPP