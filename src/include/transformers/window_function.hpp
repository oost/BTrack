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

  WindowTransformer(std::size_t windowSize)
      : BufferedTransformer(windowSize), window_(windowSize, 0) {}
  virtual ~WindowTransformer() {}

protected:
  void process() override {

    if (inputBuffer_->size() != window_.size()) {
      throw std::range_error("Wrong size");
    }

    if (inputBuffer_->size() != outputBuffer_->size()) {
      throw std::range_error("Wrong size");
    }

    for (int i = 0; i < window_.size(); i++) {
      (*outputBuffer_)[i] = (*inputBuffer_)[i] * window_[i];
    }
  }

  std::vector<double> window_;
};

} // namespace transformers

#endif // BTRACK__WINDOW_HPP