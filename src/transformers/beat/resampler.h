#ifndef BTRACK__SRC__TRANSFORMERS__BEAT__RESAMPLER__H_
#define BTRACK__SRC__TRANSFORMERS__BEAT__RESAMPLER__H_

#include <memory>
#include <samplerate.h>

#include "all.h"
#include "transformers/buffers/all.h"
#include "transformers/transformers/all.h"

namespace transformers {

/**
 * @brief Resamples ODF
 * Resamples the onset detection function from an arbitrary number of
 * samples to 512
 *
 * @param[out] onset_df_buffer            The onset detection function sample
 * buffer.
 * @param[in]  resampled_onset_df_buffer_ The resampled onset detection
 * function sample buffer
 */

class Resampler
    : public GenericTransformer<CircularBuffer<double>, ArrayBuffer<double>> {
public:
  Resampler(std::size_t output_len = 512)
      : GenericTransformer(), output_len_{output_len},
        resampler_output_(output_len), resampler_input_(output_len) {
    output_buffer_ = std::make_shared<ArrayBuffer<double>>(output_len);
  }

protected:
  void process() override {

    for (int i = 0; i < input_buffer_->size(); i++) {
      resampler_input_[i] = static_cast<float>((*input_buffer_)[i]);
    }

    double src_ratio = static_cast<double>(output_len_) /
                       static_cast<double>(input_buffer_->size());
    SRC_DATA src_data;
    src_data.data_in = resampler_input_.data();
    src_data.input_frames = input_buffer_->size();
    src_data.src_ratio = src_ratio;
    src_data.data_out = resampler_output_.data();
    src_data.output_frames = resampler_output_.size();

    src_simple(&src_data, SRC_SINC_BEST_QUALITY, 1);

    for (int i = 0; i < output_len_; i++) {
      (*output_buffer_)[i] = src_data.data_out[i];
    }
  }

  std::size_t output_len_;

  // Internal state
  std::vector<float> resampler_output_;
  std::vector<float> resampler_input_;
};

} // namespace transformers

#endif // BTRACK__SRC__TRANSFORMERS__BEAT__RESAMPLER__H_