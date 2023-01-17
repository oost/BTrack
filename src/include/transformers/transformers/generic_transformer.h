#ifndef BTRACK__SRC__INCLUDE__TRANSFORMERS__TRANSFORMERS__GENERIC_TRANSFORMER__H_
#define BTRACK__SRC__INCLUDE__TRANSFORMERS__TRANSFORMERS__GENERIC_TRANSFORMER__H_

#include <fmt/format.h>

#include "transformer.h"

namespace transformers {

template <typename IB, typename OB>
class GenericTransformer : public Transformer {
public:
  using output_value_t = typename OB::value_t;
  using output_buffer_t = OB;
  using output_buffer_ptr = std::shared_ptr<output_buffer_t>;
  using input_buffer_t = IB;
  using input_buffer_ptr = std::shared_ptr<input_buffer_t>;

  GenericTransformer() : Transformer() {}

  Buffer::Ptr output() const override {
    return std::static_pointer_cast<Buffer>(output_buffer_);
  };

  output_buffer_ptr output_cast() { return output_buffer_; }

  void set_input(Buffer::Ptr input_buffer) override {
    input_buffer_ = std::dynamic_pointer_cast<input_buffer_t>(input_buffer);
    if (input_buffer_ == nullptr) {
      throw std::runtime_error(fmt::format(
          "Chained incompatible transformers. Can't cast {} to {}",
          typeid(input_buffer).name(), typeid(input_buffer_).name()));
    }
    input_updated();
  }

protected:
  output_buffer_ptr output_buffer_;
  input_buffer_ptr input_buffer_;
};

} // namespace transformers

#endif // BTRACK__SRC__INCLUDE__TRANSFORMERS__TRANSFORMERS__GENERIC_TRANSFORMER__H_