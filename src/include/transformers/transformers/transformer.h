#ifndef BTRACK__TRANSFORMERS__TRANSFORMER_HPP
#define BTRACK__TRANSFORMERS__TRANSFORMER_HPP

#include <memory>
#include <set>
#include <span>

#include "transformers/buffers/all.h"

namespace transformers {

class TransformerPipeline;

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
    execute_sinks();
  }

  const std::set<Ptr> sinks() const { return sinks_; }

  // friend Transformer &operator<<(Transformer &source, Transformer::Ptr sink);

protected:
  virtual void process() = 0;
  void execute_sinks() {
    for (auto &sink : sinks_) {
      sink->execute();
    }
  }

  virtual void input_updated() {}

  std::set<Ptr> sinks_;

  friend class TransformerPipeline;
};

Transformer::Ptr operator>>(Transformer::Ptr source, Transformer::Ptr sink);

} // namespace transformers

#endif // BTRACK__TRANSFORMERS__TRANSFORMER_HPP