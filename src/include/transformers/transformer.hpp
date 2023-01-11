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

  virtual void set_input(Buffer::Ptr input_buffer) = 0;

  virtual void add_sink(Ptr sink) {
    sinks_.insert(sink);
    sink->set_input(output());
  }

  void execute() {
    // need to check that inputBuffer exists first...
    process();
    process_sinks();
  }

protected:
  virtual void process() = 0;
  void process_sinks() {
    for (auto &sink : sinks_) {
      sink->execute();
    }
  }

  std::set<Ptr> sinks_;
};

template <typename IB, typename OB>
class GenericTransformer : public Transformer {
public:
  using output_value_t = typename OB::value_t;
  using output_buffer_t = OB;
  using output_buffer_ptr = std::shared_ptr<output_buffer_t>;
  using input_buffer_t = IB;
  using input_buffer_ptr = std::shared_ptr<input_buffer_t>;

  GenericTransformer() : Transformer() {}

  void add_sink(Ptr sink) override {
    Transformer::add_sink(sink);
    sink->set_input(output());
  }

  Buffer::Ptr output() const override {
    return std::static_pointer_cast<Buffer>(output_buffer_);
  };

  output_buffer_ptr output_data() { return output_buffer_; }

  void set_input(Buffer::Ptr input_buffer) override {
    input_buffer_ = std::dynamic_pointer_cast<input_buffer_t>(input_buffer);
    if (input_buffer_ == nullptr) {
      throw std::runtime_error("Chained imcompatible transformers");
    }
  }

protected:
  output_buffer_ptr output_buffer_;
  input_buffer_ptr input_buffer_;
};

template <typename I, typename O>
class BufferedTransformer
    : public GenericTransformer<DataBuffer<I>, DataBuffer<O>> {
public:
  BufferedTransformer(std::size_t buffer_size)
      : GenericTransformer<DataBuffer<I>, DataBuffer<O>>() {
    this->output_buffer_ = std::make_shared<DataBuffer<O>>(buffer_size);
  }
};

template <typename I, typename O>
class ReductionTransformer
    : public GenericTransformer<DataBuffer<I>, SingleValueBuffer<O>> {
public:
  ReductionTransformer()
      : GenericTransformer<DataBuffer<I>, SingleValueBuffer<O>>() {
    this->output_buffer_ = std::make_shared<SingleValueBuffer<O>>();
  }
};

template <typename I, typename O>
class BufferedTransformer_old : public Transformer {
public:
  using output_buffer_t = DataBuffer<O>;
  using output_buffer_ptr = std::shared_ptr<output_buffer_t>;
  using input_buffer_t = DataBuffer<I>;
  using input_buffer_ptr = std::shared_ptr<input_buffer_t>;

  BufferedTransformer_old(std::size_t buffer_size) : Transformer() {
    output_buffer_ = std::make_shared<output_buffer_t>(buffer_size);
  }

  void add_sink(Ptr sink) override {
    Transformer::add_sink(sink);
    sink->set_input(output());
  }

  Buffer::Ptr output() const override {
    return std::static_pointer_cast<Buffer>(output_buffer_);
  };

  output_buffer_ptr output_data() { return output_buffer_; }

  void set_input(Buffer::Ptr input_buffer) override {
    input_buffer_ = std::dynamic_pointer_cast<input_buffer_t>(input_buffer);
    if (input_buffer_ == nullptr) {
      throw std::runtime_error("Chained imcompatible transformers");
    }
  }

protected:
  output_buffer_ptr output_buffer_;
  input_buffer_ptr input_buffer_;
};

template <typename I, typename O>
class ReductionTransformer_old : public Transformer {
public:
  using output_buffer_t = SingleValueBuffer<O>;
  using output_buffer_ptr = std::shared_ptr<output_buffer_t>;
  using input_buffer_t = DataBuffer<I>;
  using input_buffer_ptr = std::shared_ptr<input_buffer_t>;

  ReductionTransformer_old(std::size_t buffer_size) : Transformer() {
    output_buffer_ = std::make_shared<output_buffer_t>(buffer_size);
  }

  void add_sink(Ptr sink) override {
    Transformer::add_sink(sink);
    sink->set_input(output());
  }

  Buffer::Ptr output() const override {
    return std::static_pointer_cast<Buffer>(output_buffer_);
  };

  output_buffer_ptr output_data() { return output_buffer_; }

  void set_input(Buffer::Ptr input_buffer) override {
    input_buffer_ = std::dynamic_pointer_cast<input_buffer_t>(input_buffer);
    if (input_buffer_ == nullptr) {
      throw std::runtime_error("Chained imcompatible transformers");
    }
  }

protected:
  output_buffer_ptr output_buffer_;
  input_buffer_ptr input_buffer_;
};

} // namespace transformers

#endif // BTRACK__TRANSFORMERS__TRANSFORMER_HPP