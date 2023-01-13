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
#include <span>

#include "transformers/buffer.hpp"
#include "transformers/detection_function.hpp"
#include "transformers/transformer_pipeline.hpp"
#include "transformers/window_function.hpp"

using transformers::DataBuffer;
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
   * @param hop_size the hop size in audio samples
   * @param frame_size the frame size in audio samples
   */
  OnsetDetectionFunction(int hop_size, int frame_size);

  /** Constructor
   * @param hop_size the hop size in audio samples
   * @param frame_size the frame size in audio samples
   * @param onset_detection_function_type the type of onset detection function
   * to use - (see Onset_detection_function_type)
   * @param window_type the type of window to use (see WindowType)
   */
  OnsetDetectionFunction(int hop_size, int frame_size,
                         DetectionFunctionType onset_detection_function_type,
                         WindowType window_type);

  /** Constructor (for backward compatibility)
   * @param hop_size the hop size in audio samples
   * @param frame_size the frame size in audio samples
   * @param onset_detection_function_type the type of onset detection function
   * to use - (see Onset_detection_function_type)
   * @param window_type the type of window to use (see WindowType)
   */
  OnsetDetectionFunction(int hop_size, int frame_size,
                         int onset_detection_function_type, int window_type);

  /** Destructor */
  ~OnsetDetectionFunction();

  /** Process input frame and calculate detection function sample
   * @param buffer a pointer to an array containing the audio samples to be
   * processed
   * @returns the onset detection function sample
   */
  double calculate_onset_detection_function_sample(std::span<double> buffer);
  double calculate_onset_detection_function_sample(double *buffer);

private:
  int frame_size_; /**< audio frame_size */
  int hop_size_;   /**< audio hop_size */

  //=======================================================================
  TransformerPipeline<DataBuffer<double>>::Ptr pipeline_;

  //=======================================================================
  std::shared_ptr<RealDataBuffer> buffer_;
};

#endif