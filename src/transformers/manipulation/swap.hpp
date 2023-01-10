#ifndef BTRACK__TRANSFORMERS__SWAP_HPP
#define BTRACK__TRANSFORMERS__SWAP_HPP

#include <algorithm>
#include <numeric>
#include <span>
#include <vector>

#include "transformers/transformer.hpp"

namespace transformers {

template <typename T> class SwapTransformer : public BufferedTransformer<T, T> {
public:
  SwapTransformer(std::size_t spanSize) : BufferedTransformer<T, T>(spanSize) {
    if (spanSize % 2) {
      throw std::range_error("Frame size should be even");
    }
  }

protected:
  void process() override {
    if (this->input_buffer_->size() != this->output_buffer_->size()) {
      throw std::range_error("Frame size should be even");
    }

    int half_size = (this->input_buffer_->size() / 2);

    for (int i = 0; i < half_size; i++) {
      (*this->output_buffer_)[i] = (*this->input_buffer_)[half_size + i];
      (*this->output_buffer_)[i + half_size] = (*this->input_buffer_)[i];
    }
  };
};

} // namespace transformers

#endif // BTRACK__TRANSFORMERS__SWAP_HPP