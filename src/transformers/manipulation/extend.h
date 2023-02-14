#ifndef BTRACK__SRC__TRANSFORMERS__MANIPULATION__EXTEND__H_
#define BTRACK__SRC__TRANSFORMERS__MANIPULATION__EXTEND__H_

#include <range/v3/algorithm/copy.hpp>
#include <range/v3/algorithm/fill.hpp>

#include "transformers/transformers/all.h"

namespace btrack::transformers {

template <typename I, typename O>
class ExtendTransformer : public BufferedTransformer<I, O> {
public:
  ExtendTransformer(std::size_t output_size)
      : BufferedTransformer<I, O>(output_size) {}

protected:
  void input_updated() override {
    if (this->input_buffer_->size() > this->output_buffer_->size()) {
      throw std::range_error("Frame size should be less of same size");
    }
  }
  void process() override {

    ranges::copy(this->input_buffer_->data(),
                 this->output_buffer_->data().begin());
    ranges::fill(this->output_buffer_->data().begin() +
                     this->input_buffer_->size(),
                 this->output_buffer_->data().end(), 0.0);
  }
};
} // namespace btrack::transformers

#endif // BTRACK__SRC__TRANSFORMERS__MANIPULATION__EXTEND__H_