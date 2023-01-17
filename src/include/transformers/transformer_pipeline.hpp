#ifndef BTRACK__TRANSFORM__TRANSFORMER_PIPELINE_HPP
#define BTRACK__TRANSFORM__TRANSFORMER_PIPELINE_HPP

#include <concepts>
#include <memory>

#include "buffers/all.h"
#include "transformers/all.h"

namespace transformers {

class TransformerPipeline {
public:
  using Ptr = std::shared_ptr<TransformerPipeline>;

  TransformerPipeline() {}

  Buffer::Ptr output() const {
    if (final_transform_) {
      return final_transform_->output();
    }
    return nullptr;
  };

  void set_input(Buffer::Ptr input) {
    input_buffer_ = input;
    initial_transform_->set_input(input);
  }

  template <class OB> typename OB::Ptr output_cast() const {
    return std::dynamic_pointer_cast<OB>(final_transform_->output());
  }

  template <class IB> typename IB::Ptr input_cast() const {
    return std::dynamic_pointer_cast<IB>(input_buffer_);
  }

  template <class TP>
  void set_initial_transform(TP initial_transform)
    requires std::derived_from<typename TP::element_type, Transformer>
  {
    initial_transform_ =
        std::static_pointer_cast<Transformer>(initial_transform);

    final_transform_ = get_final_transformer(initial_transform);
  }

  void execute() { initial_transform_->execute(); }

private:
  Transformer::Ptr get_final_transformer(Transformer::Ptr current) {
    if (!(current->sinks_.size())) {
      return current;
    }
    Transformer::Ptr final = current;
    // DFS to check that there is only one final transformer
    for (auto next : current->sinks_) {
      auto next_final = get_final_transformer(next);

      // If final has already been set then there is a sibling. Do we have the
      // same final?
      if ((final != current) and (next_final != final)) {
        throw std::runtime_error("Pipeline with multiple final transforms");
      }
      final = next_final;
    }
    return final;
  }

  Buffer::Ptr input_buffer_;
  Transformer::Ptr initial_transform_;
  Transformer::Ptr final_transform_;
};

} // namespace transformers

#endif // BTRACK__TRANSFORM__TRANSFORMER_PIPELINE_HPP