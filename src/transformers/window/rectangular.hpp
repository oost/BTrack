#ifndef BTRACK__WINDOW_FUNCTIONS__RECTANGULAR_HPP
#define BTRACK__WINDOW_FUNCTIONS__RECTANGULAR_HPP

#include "transformers/window_function.hpp"

namespace transformers {

class RectangularWindowTransformer : public WindowTransformer {
public:
  RectangularWindowTransformer(std::size_t windowSize)
      : WindowTransformer(windowSize) {
    // Rectangular window calculation
    for (int n = 0; n < windowSize; n++) {
      window_[n] = 1.0;
    }
  }
};

} // namespace transformers

#endif // BTRACK__WINDOW_FUNCTIONS__RECTANGULAR_HPP
