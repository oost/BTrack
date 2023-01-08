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

#include "onset_detection_function.hpp"
#include "transformers/fft_operator.hpp"
#include "transformers/manipulation/all.hpp"
#include "transformers/onset_detection/all.hpp"
#include "transformers/window/all.hpp"

using transformers::FFTOperator;
using transformers::MapTransformer;
using transformers::ShiftTransformer;
using transformers::Transformer;

//=======================================================================
OnsetDetectionFunction::OnsetDetectionFunction(int hopSize, int frameSize)
    : OnsetDetectionFunction{
          hopSize, frameSize,
          DetectionFunctionType::ComplexSpectralDifferenceHWR,
          WindowType::HanningWindow} {}

//=======================================================================
OnsetDetectionFunction::OnsetDetectionFunction(
    int hopSize, int frameSize,
    DetectionFunctionType onsetDetectionFunctionType, WindowType windowType) {

  hopSize_ = hopSize;     // set hopsize
  frameSize_ = frameSize; // set framesize

  // buffer_ = std::make_shared<RealDataBuffer>(hopSize);

  pipeline_ = std::make_shared<TransformerPipeline<double>>(hopSize);
  buffer_ = pipeline_->inputBuffer();

  Transformer::Ptr shifter =
      std::make_shared<ShiftTransformer<double>>(frameSize);
  pipeline_->setInitialTransform(shifter);

  Transformer::Ptr window = createWindowTransformer(windowType, frameSize);
  shifter->addSink(window);

  Transformer::Ptr mapper =
      std::make_shared<MapTransformer<double, std::complex<double>>>(
          frameSize, [](double v) { return std::complex<double>(v, 0); });
  window->addSink(mapper);

  FFTOperator::Ptr fft = FFTOperator::createOperator(frameSize, false);
  mapper->addSink(std::static_pointer_cast<Transformer>(fft));

  Transformer::Ptr odf =
      createDetectionFunction(onsetDetectionFunctionType, frameSize);
  fft->addSink(odf);

  pipeline_->setFinalTransform(odf);
}

//=======================================================================
OnsetDetectionFunction::OnsetDetectionFunction(int hopSize, int frameSize,
                                               int onsetDetectionFunctionType,
                                               int windowType)
    : OnsetDetectionFunction{
          hopSize, frameSize,
          static_cast<DetectionFunctionType>(onsetDetectionFunctionType),
          static_cast<WindowType>(windowType)} {}

//=======================================================================
OnsetDetectionFunction::~OnsetDetectionFunction() {}

//=======================================================================

double
OnsetDetectionFunction::calculateOnsetDetectionFunctionSample(double *buffer) {
  std::vector<double> buf_new(buffer, buffer + hopSize_);
  return calculateOnsetDetectionFunctionSample(buf_new);
}

double OnsetDetectionFunction::calculateOnsetDetectionFunctionSample(
    std::vector<double> &input) {

  std::copy(input.begin(), input.end(), buffer_->data().begin());

  pipeline_->execute();

  RealDataBuffer &output =
      *std::dynamic_pointer_cast<RealDataBuffer>(pipeline_->output());

  return output[0];
}
