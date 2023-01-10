#ifndef BTRACK__WINDOW_FUNCTIONS__TUKEY_HPP
#define BTRACK__WINDOW_FUNCTIONS__TUKEY_HPP

#include "transformers/window_function.hpp"

namespace transformers {

class TukeyWindowTransformer : public WindowTransformer {
public:
  TukeyWindowTransformer(std::size_t window_size)
      : WindowTransformer(window_size) {
    double N;     // variable to store window_size minus 1
    double n_val; // double version of index 'n'
    double alpha; // alpha [default value = 0.5];

    alpha = 0.5;

    N = static_cast<double>(window_size - 1); // window_size minus 1

    // Tukey window calculation

    n_val = static_cast<double>(-1 * ((window_size / 2))) + 1;

    for (int n = 0; n < window_size; n++) // left taper
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
