#ifndef BTRACK__TRANSFORM_PIPELINE_HPP
#define BTRACK__TRANSFORM_PIPELINE_HPP

#include <memory>

#include "buffer.hpp"
#include "transformer.hpp"

namespace transformers {

template <typename I> class TransformerPipeline : public Transformer {
public:
  using Ptr = std::shared_ptr<TransformerPipeline>;

  TransformerPipeline(std::size_t input_buffer_size)
      : Transformer(), input_buffer_{std::make_shared<DataBuffer<I>>(
                           input_buffer_size)} {}

  Buffer::Ptr output() const override {
    if (final_transform_) {
      return final_transform_->output();
    }
    return nullptr;
  };

  typename DataBuffer<I>::Ptr input_buffer() const { return input_buffer_; };

  void set_input(Buffer::Ptr input_buffer) override {
    input_buffer_ = std::dynamic_pointer_cast<DataBuffer<I>>(input_buffer);
    if (input_buffer_ == nullptr) {
      throw std::runtime_error("Chained imcompatible transformers");
    }
  }

  void set_initial_transform(Transformer::Ptr initial_transform) {
    initial_transform_ = initial_transform;
    initial_transform_->set_input(input_buffer_);
  }
  void set_final_transform(Transformer::Ptr final_transform) {
    final_transform_ = final_transform;
  }

private:
  void process() override { initial_transform_->execute(); }

  typename DataBuffer<I>::Ptr input_buffer_;
  Transformer::Ptr initial_transform_;
  Transformer::Ptr final_transform_;
};

} // namespace transformers

#endif // BTRACK__TRANSFORM_PIPELINE_HPP