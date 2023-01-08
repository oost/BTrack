#ifndef BTRACK__TRANSFORMERS__TRANSFORMER_HPP
#define BTRACK__TRANSFORMERS__TRANSFORMER_HPP

#include <memory>
#include <set>
#include <span>

#include "buffer.hpp"

namespace transformers {

class Transformer {
public:
  using Ptr = std::shared_ptr<Transformer>;

  virtual ~Transformer() {}

  virtual Buffer::Ptr output() const = 0;

  virtual void setInput(Buffer::Ptr inputBuffer) = 0;

  virtual void addSink(Ptr sink) {
    sinks_.insert(sink);
    sink->setInput(output());
  }

  void execute() {
    // need to check that inputBuffer exists first...
    process();
    processSinks();
  }

protected:
  virtual void process() = 0;
  void processSinks() {
    for (auto &sink : sinks_) {
      sink->execute();
    }
  }

  std::set<Ptr> sinks_;
};

template <typename I, typename O>
class BufferedTransformer : public Transformer {
public:
  using output_buffer_t = DataBuffer<O>;
  using output_buffer_ptr = std::shared_ptr<DataBuffer<O>>;
  using input_buffer_t = DataBuffer<I>;
  using input_buffer_ptr = std::shared_ptr<DataBuffer<I>>;

  BufferedTransformer(std::size_t bufferSize) : Transformer() {
    outputBuffer_ = std::make_shared<output_buffer_t>(bufferSize);
  }

  void addSink(Ptr sink) override {
    Transformer::addSink(sink);
    sink->setInput(output());
  }

  Buffer::Ptr output() const override {
    return std::static_pointer_cast<Buffer>(outputBuffer_);
  };

  output_buffer_ptr outputData() { return outputBuffer_; }

  void setInput(Buffer::Ptr inputBuffer) override {
    inputBuffer_ = std::dynamic_pointer_cast<input_buffer_t>(inputBuffer);
    if (inputBuffer_ == nullptr) {
      throw std::runtime_error("Chained imcompatible transformers");
    }
  }

protected:
  // template <typename I> DataBuffer<I> &inputData(Buffer::Ptr input) {
  //   return *std::dynamic_pointer_cast<DataBuffer<I>>(input);
  // }

  output_buffer_ptr outputBuffer_;
  input_buffer_ptr inputBuffer_;
};

} // namespace transformers

#endif // BTRACK__TRANSFORMERS__TRANSFORMER_HPP