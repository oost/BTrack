#ifndef SERVER__EXTERNAL__BTRACK__SRC__ORIGINAL__ONSETDETECTIONFUNCTIONORIGINAL_H_
#define SERVER__EXTERNAL__BTRACK__SRC__ORIGINAL__ONSETDETECTIONFUNCTIONORIGINAL_H_

#include <span>

#include "./OnsetDetectionFunction.h"
#include "transformers/transformer_pipeline.hpp"

class OnsetDetectionFunctionOriginal
    : public btrack_original::OnsetDetectionFunction,
      public btrack::transformers::TransformerPipeline {
public:
  using Ptr = std::unique_ptr<OnsetDetectionFunctionOriginal>;

  /** Constructor that defaults the onset detection function type to
   * ComplexSpectralDifferenceHWR and the window type to HanningWindow
   * @param hop_size the hop size in audio samples
   * @param frame_size the frame size in audio samples
   */
  OnsetDetectionFunctionOriginal(int hop_size, int frame_size)
      : OnsetDetectionFunction(hop_size, frame_size) {}

  /** Constructor
   * @param hop_size the hop size in audio samples
   * @param frame_size the frame size in audio samples
   * @param onset_detection_function_type the type of onset detection function
   * to use - (see Onset_detection_function_type)
   * @param window_type the type of window to use (see WindowType)
   */

  OnsetDetectionFunctionOriginal(int hop_size, int frame_size,
                                 int onset_detection_function_type,
                                 int window_type)
      : OnsetDetectionFunction{
            hop_size, frame_size,
            static_cast<btrack_original::OnsetDetectionFunctionType>(
                onset_detection_function_type),
            static_cast<btrack_original::WindowType>(window_type)} {}
};

#endif // SERVER__EXTERNAL__BTRACK__SRC__ORIGINAL__ONSETDETECTIONFUNCTIONORIGINAL_H_