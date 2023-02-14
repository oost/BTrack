#ifndef BTRACK__WINDOW_FUNCTIONS__ALL_HPP
#define BTRACK__WINDOW_FUNCTIONS__ALL_HPP

#include <memory>

#include "./blackman.hpp"
#include "./hamming.hpp"
#include "./hann.hpp"
#include "./rectangular.hpp"
#include "./tukey.hpp"
#include "transformers/window_function.hpp"

namespace btrack::transformers {

std::shared_ptr<WindowTransformer>
createWindowTransformer(WindowType windowType, std::size_t frameSize) {
  // set the window to the specified type
  switch (windowType) {
  case WindowType::RectangularWindow:
    return std::make_unique<RectangularWindowTransformer>(frameSize);

  case WindowType::HanningWindow:
    return std::make_unique<HannWindowTransformer>(frameSize);

  case WindowType::HammingWindow:
    return std::make_unique<HammingWindowTransformer>(frameSize);

  case WindowType::BlackmanWindow:
    return std::make_unique<BlackmanWindow>(frameSize);

  case WindowType::TukeyWindow:
    return std::make_unique<TukeyWindowTransformer>(frameSize);

  default:
    return std::make_unique<HannWindowTransformer>(frameSize);
  }
}

} // namespace btrack::transformers

#endif // BTRACK__WINDOW_FUNCTIONS__ALL_HPP