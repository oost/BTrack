#ifndef BTRACK__WINDOW_FUNCTIONS__RECTANGULAR_HPP
#define BTRACK__WINDOW_FUNCTIONS__RECTANGULAR_HPP

#include "transformers/window_function.hpp"

namespace btrack::transformers {

class RectangularWindowTransformer : public WindowTransformer {
public:
  RectangularWindowTransformer(std::size_t window_size)
      : WindowTransformer(window_size) {
    // Rectangular window calculation
    for (int n = 0; n < window_size; n++) {
      window_[n] = 1.0;
    }
  }
};

} // namespace btrack::transformers

#endif // BTRACK__WINDOW_FUNCTIONS__RECTANGULAR_HPP
