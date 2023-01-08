#ifndef BTRACK__TRANSFORM_PIPELINE_HPP
#define BTRACK__TRANSFORM_PIPELINE_HPP

#include <memory>

#include "buffer.hpp"
#include "transformer.hpp"

namespace transformers {

template <typename I> class TransformerPipeline : public Transformer {
public:
  using Ptr = std::shared_ptr<TransformerPipeline>;

  TransformerPipeline(std::size_t inputBufferSize)
      : Transformer(), inputBuffer_{
                           std::make_shared<DataBuffer<I>>(inputBufferSize)} {}

  Buffer::Ptr output() const override {
    if (finalTransform_) {
      return finalTransform_->output();
    }
    return nullptr;
  };

  typename DataBuffer<I>::Ptr inputBuffer() const { return inputBuffer_; };

  void setInput(Buffer::Ptr inputBuffer) override {
    inputBuffer_ = std::dynamic_pointer_cast<DataBuffer<I>>(inputBuffer);
    if (inputBuffer_ == nullptr) {
      throw std::runtime_error("Chained imcompatible transformers");
    }
  }

  void setInitialTransform(Transformer::Ptr initialTransform) {
    initialTransform_ = initialTransform;
    initialTransform_->setInput(inputBuffer_);
  }
  void setFinalTransform(Transformer::Ptr finalTransform) {
    finalTransform_ = finalTransform;
  }

private:
  void process() override { initialTransform_->execute(); }

  typename DataBuffer<I>::Ptr inputBuffer_;
  Transformer::Ptr initialTransform_;
  Transformer::Ptr finalTransform_;
};

} // namespace transformers

#endif // BTRACK__TRANSFORM_PIPELINE_HPP