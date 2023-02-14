#ifndef BTRACK__SRC__TRANSFORMERS__MANIPULATION__TAKE__H_
#define BTRACK__SRC__TRANSFORMERS__MANIPULATION__TAKE__H_

// #include <range/all.h>

#include "transformers/transformers/all.h"

namespace btrack::transformers {

template <typename I, typename O>
class TakeTransformer : public BufferedTransformer<I, O> {
public:
  TakeTransformer(std::size_t output_size)
      : BufferedTransformer<I, O>(output_size) {}

protected:
  void input_updated() override {
    if (this->input_buffer_->size() < this->output_buffer_->size()) {
      throw std::range_error("Frame size should be less of same size");
    }
  }
  void process() override {

    ranges::copy(this->input_buffer_->data().begin(),
                 this->input_buffer_->data().begin() +
                     this->output_buffer_->data().size(),
                 this->output_buffer_->data().begin());
  }
};
} // namespace btrack::transformers

#endif // BTRACK__SRC__TRANSFORMERS__MANIPULATION__TAKE__H_