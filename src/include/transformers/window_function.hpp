#ifndef BTRACK__WINDOW_HPP
#define BTRACK__WINDOW_HPP

#include "./transformers/transformer.hpp"

namespace transformers {
//=======================================================================
/** The type of window to use when calculating onset detection function samples
 */
enum class WindowType {
  RectangularWindow,
  HanningWindow,
  HammingWindow,
  BlackmanWindow,
  TukeyWindow
};

class WindowTransformer : public BufferedTransformer<double, double> {
public:
  using Ptr = std::unique_ptr<WindowTransformer>;

  WindowTransformer(std::size_t window_size)
      : BufferedTransformer(window_size), window_(window_size, 0) {}
  virtual ~WindowTransformer() {}

protected:
  void process() override {

    if (input_buffer_->size() != window_.size()) {
      throw std::range_error("Wrong size");
    }

    if (input_buffer_->size() != output_buffer_->size()) {
      throw std::range_error("Wrong size");
    }

    for (int i = 0; i < window_.size(); i++) {
      (*output_buffer_)[i] = (*input_buffer_)[i] * window_[i];
    }
  }

  std::vector<double> window_;
};

} // namespace transformers

#endif // BTRACK__WINDOW_HPP