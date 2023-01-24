//=======================================================================
/** @file OnsetDetectionFunction.cpp
 *  @brief A class for calculating onset detection functions
 *  @author Adam Stark
 *  @copyright Copyright (C) 2008-2014  Queen Mary University of London
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//=======================================================================

#include <cmath>
#include <complex>
#include <memory>
#include <numbers>
#include <numeric>

#include "onset_detection_function.h"
#include "transformers/fft_operator.hpp"
#include "transformers/manipulation/all.h"
#include "transformers/onset_detection/all.hpp"
#include "transformers/window/all.hpp"

using transformers::Buffer;
using transformers::FFTOperator;
using transformers::MapTransformer;
using transformers::ShiftTransformer;
using transformers::SingleValueBuffer;
using transformers::SwapTransformer;
using transformers::Transformer;

//=======================================================================
OnsetDetectionFunction::OnsetDetectionFunction(int hop_size, int frame_size)
    : OnsetDetectionFunction{
          hop_size, frame_size,
          DetectionFunctionType::ComplexSpectralDifferenceHWR,
          WindowType::HanningWindow} {}

//=======================================================================
OnsetDetectionFunction::OnsetDetectionFunction(
    int hop_size, int frame_size,
    DetectionFunctionType onset_detection_function_type,
    WindowType window_type) {

  hop_size_ = hop_size;     // set hop_size
  frame_size_ = frame_size; // set frame_size

  Transformer::Ptr shifter =
      std::make_shared<ShiftTransformer<double>>(frame_size);
  Transformer::Ptr window = createWindowTransformer(window_type, frame_size);
  Transformer::Ptr swapper =
      std::make_shared<SwapTransformer<double>>(frame_size);
  Transformer::Ptr mapper =
      std::make_shared<MapTransformer<double, std::complex<double>>>(
          frame_size, [](double v) { return std::complex<double>(v, 0); });

  Transformer::Ptr fft = FFTOperator::create_operator(frame_size, false);
  Transformer::Ptr odf =
      create_detection_function(onset_detection_function_type, frame_size);

  shifter >> window >> swapper >> mapper >> fft >> odf;

  set_initial_transform(shifter);
}

//=======================================================================
OnsetDetectionFunction::OnsetDetectionFunction(
    int hop_size, int frame_size, int onset_detection_function_type,
    int window_type)
    : OnsetDetectionFunction{
          hop_size, frame_size,
          static_cast<DetectionFunctionType>(onset_detection_function_type),
          static_cast<WindowType>(window_type)} {}

//=======================================================================
OnsetDetectionFunction::~OnsetDetectionFunction() {}
