#ifndef BTRACK__SRC__INCLUDE__TRANSFORMERS__TRANSFORMERS__SPECIALIZED_TRANSFORMERS__H_
#define BTRACK__SRC__INCLUDE__TRANSFORMERS__TRANSFORMERS__SPECIALIZED_TRANSFORMERS__H_

#include "generic_transformer.h"

namespace btrack::transformers {

template <typename I, typename O>
class BufferedTransformer
    : public GenericTransformer<ArrayBuffer<I>, ArrayBuffer<O>> {
public:
  BufferedTransformer(std::size_t buffer_size)
      : GenericTransformer<ArrayBuffer<I>, ArrayBuffer<O>>() {
    this->output_buffer_ = std::make_shared<ArrayBuffer<O>>(buffer_size);
  }
};

template <typename I, typename O>
class ReductionTransformer
    : public GenericTransformer<ArrayBuffer<I>, SingleValueBuffer<O>> {
public:
  ReductionTransformer()
      : GenericTransformer<ArrayBuffer<I>, SingleValueBuffer<O>>() {
    this->output_buffer_ = std::make_shared<SingleValueBuffer<O>>();
  }
};

template <typename I, typename O>
class AccumulatorTransformer
    : public GenericTransformer<SingleValueBuffer<I>, ArrayBuffer<O>> {
public:
  AccumulatorTransformer()
      : GenericTransformer<SingleValueBuffer<I>, ArrayBuffer<O>>() {
    this->output_buffer_ = std::make_shared<ArrayBuffer<O>>();
  }
};

template <typename IB>
class MultiTransformer : public GenericTransformer<IB, MultiBuffer> {
public:
  MultiTransformer() : GenericTransformer<IB, MultiBuffer>() {
    this->output_buffer_ = std::make_shared<MultiBuffer>();
  }
};

} // namespace btrack::transformers

#endif // BTRACK__SRC__INCLUDE__TRANSFORMERS__TRANSFORMERS__SPECIALIZED_TRANSFORMERS__H_