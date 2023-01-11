#ifndef BTRACK__BEAT__BEAT_PREDICTOR_HPP
#define BTRACK__BEAT__BEAT_PREDICTOR_HPP

#include "transformers/buffer.hpp"
#include "transformers/transformer.hpp"
#include <memory>

namespace transformers {

template <typename I>
class BeatPredictor : public ReductionTransformer<I, MultiBuffer> {
public:
  const char *beat_period_id = "beat_period";
  const char *cumulative_score_id_ = "cumulative_score";
  const char *m0_id_ = "m0";
  const char *beat_counter_id_ = "beat_counter";

  BeatPredictor(std::size_t onset_df_buffer_size, double tightness)
      : ReductionTransformer<I, double>() {

    onset_df_buffer_size_ = onset_df_buffer_size;
    tightness_ = tightness;

    // Inputs
    std::shared_ptr<MultiBuffer> input_buffer = std::make_shared<MultiBuffer>();
    beat_period_ = std::dynamic_pointer_cast<DataBuffer<double>>(
        input_buffer->buffer(beat_period_id));
    cumulative_score_ = std::dynamic_pointer_cast<DataBuffer<double>>(
        input_buffer->buffer(cumulative_score_id_));

    // Outputs
    m0_ = std::make_shared<SingleValueBuffer<double>>();
    beat_counter_ = std::make_shared<SingleValueBuffer<double>>();
    this->output_buffer_->add_buffer(m0_id_, m0_);
    this->output_buffer_->add_buffer(beat_counter_id_, beat_counter_);
  }

protected:
  void process() override {

    // output_buffer_->value() = sum;
    double beat_period = beat_period_->value();
    int window_size = static_cast<int>(beat_period);
    std::vector<double> future_cumulative_score(onset_df_buffer_size_ +
                                                window_size);
    std::vector<double> w2(window_size);

    // copy cumscore to first part of fcumscore
    for (int i = 0; i < onset_df_buffer_size_; i++) {
      future_cumulative_score[i] = (*cumulative_score_)[i];
    }

    // create future window
    double v = 1;
    for (int i = 0; i < window_size; i++) {
      w2[i] = exp((-1 * pow((v - (beat_period / 2)), 2)) /
                  (2 * pow((beat_period / 2), 2)));
      v++;
    }

    // create past window
    v = -2 * beat_period;
    int start = onset_df_buffer_size_ - round(2 * beat_period);
    int end = onset_df_buffer_size_ - round(beat_period / 2);
    int pastwinsize = end - start + 1;
    std::vector<double> w1(pastwinsize);

    for (int i = 0; i < pastwinsize; i++) {
      w1[i] = exp((-1 * pow(tightness_ * log(-v / beat_period), 2)) / 2);
      v = v + 1;
    }

    // calculate future cumulative score
    double max;
    int n;
    double wcumscore;
    for (int i = onset_df_buffer_size_;
         i < (onset_df_buffer_size_ + window_size); i++) {
      start = i - round(2 * beat_period);
      end = i - round(beat_period / 2);

      max = 0;
      n = 0;
      for (int k = start; k <= end; k++) {
        wcumscore = future_cumulative_score[k] * w1[n];

        if (wcumscore > max) {
          max = wcumscore;
        }
        n++;
      }

      future_cumulative_score[i] = max;
    }

    // predict beat
    max = 0;
    n = 0;

    for (int i = onset_df_buffer_size_;
         i < (onset_df_buffer_size_ + window_size); i++) {
      wcumscore = future_cumulative_score[i] * w2[n];

      if (wcumscore > max) {
        max = wcumscore;
        beat_counter_ = n;
      }

      n++;
    }

    // set next prediction time
    m0_->value() = beat_counter_ + round(beat_period / 2);
  }

  std::size_t onset_df_buffer_size_;
  double tightness_;

  SingleValueBuffer<double>::Ptr beat_period_;
  DataBuffer<double>::Ptr cumulative_score_;

  SingleValueBuffer<double>::Ptr m0_;
  SingleValueBuffer<double>::Ptr beat_counter_;
};

} // namespace transformers

#endif // BTRACK__BEAT__BEAT_PREDICTOR_HPP