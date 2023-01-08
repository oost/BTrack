//=======================================================================
/** @file OnsetDetectionFunction.h
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

#ifndef __ONSETDETECTIONFUNCTION_H
#define __ONSETDETECTIONFUNCTION_H

#include <memory>
#include <vector>

#include "transformers/buffer.hpp"
#include "transformers/detection_function.hpp"
#include "transformers/transformer_pipeline.hpp"
#include "transformers/window_function.hpp"

using transformers::DetectionFunctionType;
using transformers::RealDataBuffer;
using transformers::TransformerPipeline;
using transformers::WindowType;

//=======================================================================
/** A class for calculating onset detection functions. */
class OnsetDetectionFunction {
public:
  using Ptr = std::unique_ptr<OnsetDetectionFunction>;

  /** Constructor that defaults the onset detection function type to
   * ComplexSpectralDifferenceHWR and the window type to HanningWindow
   * @param hopSize the hop size in audio samples
   * @param frameSize the frame size in audio samples
   */
  OnsetDetectionFunction(int hopSize, int frameSize);

  /** Constructor
   * @param hopSize the hop size in audio samples
   * @param frameSize the frame size in audio samples
   * @param onsetDetectionFunctionType the type of onset detection function to
   * use - (see OnsetDetectionFunctionType)
   * @param windowType the type of window to use (see WindowType)
   */
  OnsetDetectionFunction(int hopSize, int frameSize,
                         DetectionFunctionType onsetDetectionFunctionType,
                         WindowType windowType);

  /** Constructor (for backward compatibility)
   * @param hopSize the hop size in audio samples
   * @param frameSize the frame size in audio samples
   * @param onsetDetectionFunctionType the type of onset detection function to
   * use - (see OnsetDetectionFunctionType)
   * @param windowType the type of window to use (see WindowType)
   */
  OnsetDetectionFunction(int hopSize, int frameSize,
                         int onsetDetectionFunctionType, int windowType);

  /** Destructor */
  ~OnsetDetectionFunction();

  /** Process input frame and calculate detection function sample
   * @param buffer a pointer to an array containing the audio samples to be
   * processed
   * @returns the onset detection function sample
   */
  double calculateOnsetDetectionFunctionSample(std::vector<double> &buffer);
  double calculateOnsetDetectionFunctionSample(double *buffer);

private:
  int frameSize_; /**< audio framesize */
  int hopSize_;   /**< audio hopsize */

  //=======================================================================
  TransformerPipeline<double>::Ptr pipeline_;

  //=======================================================================
  std::shared_ptr<RealDataBuffer> buffer_;
};

#endif