#ifndef BTRACK__BEAT__BEAT_PREDICTOR_HPP
#define BTRACK__BEAT__BEAT_PREDICTOR_HPP

#include "transformers/buffer.hpp"
#include "transformers/transformer.hpp"
#include <memory>

namespace transformers {

class BeatPredictor : public MultiTransformer {
public:
  static constexpr char const *beat_period_id = "beat_period";
  static constexpr char const *cumulative_score_id = "cumulative_score";

  static constexpr char const *m0_id = "m0";
  static constexpr char const *beat_counter_id = "beat_counter";

  BeatPredictor(double tightness) : MultiTransformer() {
    tightness_ = tightness;

    // Inputs
    std::shared_ptr<MultiBuffer> input_buffer = std::make_shared<MultiBuffer>();
    beat_period_ =
        input_buffer->buffer_cast<SingleValueBuffer<double>>(beat_period_id);
    cumulative_score_ =
        input_buffer->buffer_cast<DataBuffer<double>>(cumulative_score_id);

    // Outputs
    m0_ = std::make_shared<SingleValueBuffer<double>>();
    beat_counter_ = std::make_shared<SingleValueBuffer<double>>();
    this->output_buffer_->add_buffer(m0_id, m0_);
    this->output_buffer_->add_buffer(beat_counter_id, beat_counter_);
  }

protected:
  void process() override {

    // output_buffer_->value() = sum;
    double beat_period = beat_period_->value();
    int window_size = static_cast<int>(beat_period);
    std::size_t cumulative_score_len = cumulative_score_->size();
    std::vector<double> future_cumulative_score(cumulative_score_len +
                                                window_size);
    std::vector<double> w2(window_size);

    // copy cumscore to first part of fcumscore
    for (int i = 0; i < cumulative_score_len; i++) {
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
    int start = cumulative_score_len - round(2 * beat_period);
    int end = cumulative_score_len - round(beat_period / 2);
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
    for (int i = cumulative_score_len; i < (cumulative_score_len + window_size);
         i++) {
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

    for (int i = cumulative_score_len; i < (cumulative_score_len + window_size);
         i++) {
      wcumscore = future_cumulative_score[i] * w2[n];

      if (wcumscore > max) {
        max = wcumscore;
        beat_counter_->value() = n;
      }

      n++;
    }

    // set next prediction time
    m0_->value() = beat_counter_->value() + round(beat_period / 2);
  }

  double tightness_;

  SingleValueBuffer<double>::Ptr beat_period_;
  DataBuffer<double>::Ptr cumulative_score_;

  SingleValueBuffer<double>::Ptr m0_;
  SingleValueBuffer<double>::Ptr beat_counter_;
};

} // namespace transformers

#endif // BTRACK__BEAT__BEAT_PREDICTOR_HPP