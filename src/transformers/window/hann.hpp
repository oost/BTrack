#ifndef BTRACK__WINDOW_FUNCTIONS__HANN_HPP
#define BTRACK__WINDOW_FUNCTIONS__HANN_HPP

#include "transformers/window_function.hpp"

namespace transformers {

class HannWindowTransformer : public WindowTransformer {
public:
  HannWindowTransformer(std::size_t windowSize)
      : WindowTransformer(windowSize) {
    double N = static_cast<double>(windowSize - 1); // frameSize_ minus 1

    // Hanning window calculation
    for (int n = 0; n < windowSize; n++) {
      window_[n] =
          0.5 * (1 - cos(2 * std::numbers::pi * (static_cast<double>(n) / N)));
    }
  }
};

} // namespace transformers

#endif // BTRACK__WINDOW_FUNCTIONS__HANN_HPP