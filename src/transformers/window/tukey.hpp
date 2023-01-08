#ifndef BTRACK__WINDOW_FUNCTIONS__TUKEY_HPP
#define BTRACK__WINDOW_FUNCTIONS__TUKEY_HPP

#include "transformers/window_function.hpp"

namespace transformers {

class TukeyWindowTransformer : public WindowTransformer {
public:
  TukeyWindowTransformer(std::size_t windowSize)
      : WindowTransformer(windowSize) {
    double N;     // variable to store windowSize minus 1
    double n_val; // double version of index 'n'
    double alpha; // alpha [default value = 0.5];

    alpha = 0.5;

    N = static_cast<double>(windowSize - 1); // windowSize minus 1

    // Tukey window calculation

    n_val = static_cast<double>(-1 * ((windowSize / 2))) + 1;

    for (int n = 0; n < windowSize; n++) // left taper
    {
      if ((n_val >= 0) && (n_val <= (alpha * (N / 2)))) {
        window_[n] = 1.0;
      } else if ((n_val <= 0) && (n_val >= (-1 * alpha * (N / 2)))) {
        window_[n] = 1.0;
      } else {
        window_[n] =
            0.5 *
            (1 + cos(std::numbers::pi * (((2 * n_val) / (alpha * N)) - 1)));
      }

      n_val = n_val + 1;
    }
  }
};

} // namespace transformers

#endif // BTRACK__WINDOW_FUNCTIONS__TUKEY_HPP
