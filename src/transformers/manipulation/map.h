#ifndef BTRACK__TRANSFORMERS__MAP_HPP
#define BTRACK__TRANSFORMERS__MAP_HPP

#include <complex>
#include <range/v3/algorithm/transform.hpp>
#include <span>

#include "transformers/transformers/all.h"

namespace btrack::transformers {

template <typename I, typename O>
class MapTransformer : public BufferedTransformer<I, O> {
public:
  MapTransformer(std::size_t frame_size, auto fn)
      : BufferedTransformer<I, O>(frame_size), fn_{fn} {}

protected:
  void input_updated() override {
    if (this->input_buffer_->size() != this->output_buffer_->size()) {
      throw std::range_error("Frame size should be less of same size");
    }
  }

  void process() override {
    ranges::transform(this->input_buffer_->data(),
                      this->output_buffer_->data().begin(), fn_);
    // copy into complex array and zero pad
    // for (int i = 0; i < this->input_buffer_->size(); i++) {
    //   (*this->output_buffer_)[i] = fn_((*this->input_buffer_)[i]);
    // }
  }

  std::function<O(I)> fn_;
};
} // namespace btrack::transformers

#endif // BTRACK__TRANSFORMERS__MAP_HPP