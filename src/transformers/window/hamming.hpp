#ifndef BTRACK__WINDOW_FUNCTIONS__HAMMING_HPP
#define BTRACK__WINDOW_FUNCTIONS__HAMMING_HPP

#include "transformers/window_function.hpp"

namespace transformers {

class HammingWindowTransformer : public WindowTransformer {
public:
  HammingWindowTransformer(std::size_t windowSize)
      : WindowTransformer(windowSize) {
    double N;     // variable to store framesize minus 1
    double n_val; // double version of index 'n'

    N = static_cast<double>(windowSize - 1); // frameSize_ minus 1
    n_val = 0;

    // Hamming window calculation
    for (int n = 0; n < windowSize; n++) {
      window_[n] = 0.54 - (0.46 * cos(2 * std::numbers::pi * (n_val / N)));
      n_val = n_val + 1;
    }
  }
};
} // namespace transformers

#endif // BTRACK__WINDOW_FUNCTIONS__HAMMING_HPP
