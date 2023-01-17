#ifndef BTRACK__BEAT__BEAT_PREDICTOR_HPP
#define BTRACK__BEAT__BEAT_PREDICTOR_HPP

#include <memory>

#include "all.h"
#include "transformers/buffers/all.h"
#include "transformers/transformers/all.h"

namespace transformers {

/**
 * @brief Beat predictor
 * Predicts the next beat, based upon the internal program state
 * @param[out] m0               The memory area to copy to.
 * @param[out] beat_counter     The memory area to copy to.
 * @param[in]  beat_period      The memory area to copy from.
 * @param[in]  cumulative_score The number of bytes to copy *
 */

class BeatPredictor : public MultiTransformer<MultiBuffer> {
public:
  BeatPredictor(double tightness) : MultiTransformer() {
    tightness_ = tightness;

    // Outputs
    m0_ = std::make_shared<SingleValueBuffer<int>>();
    beat_counter_ = std::make_shared<SingleValueBuffer<int>>();

    this->output_buffer_->add_buffer(transformers::constants::m0_id, m0_);
    this->output_buffer_->add_buffer(transformers::constants::beat_counter_id,
                                     beat_counter_);
  }

protected:
  void input_updated() override {
    // Inputs
    beat_period_ = input_buffer_->buffer_cast<SingleValueBuffer<double>>(
        transformers::constants::beat_period_id);
    cumulative_score_ = input_buffer_->buffer_cast<CircularBuffer<double>>(
        transformers::constants::cumulative_score_id);
  }

  void process() override {
    double beat_period = beat_period_->value();
    int window_size = static_cast<int>(beat_period);
    std::size_t cumulative_score_len = cumulative_score_->size();
    std::vector<double> future_cumulative_score(
        cumulative_score_len + window_size, 0.0);
    std::vector<double> w2(window_size);

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
        beat_counter_->set_value(n);
      }

      n++;
    }

    // set next prediction time
    m0_->set_value(beat_counter_->value() + round(beat_period / 2));
  }

  double tightness_;

  SingleValueBuffer<double>::CPtr beat_period_;
  CircularBuffer<double>::Ptr cumulative_score_;

  SingleValueBuffer<int>::Ptr m0_;
  SingleValueBuffer<int>::Ptr beat_counter_;
};

} // namespace transformers

#endif // BTRACK__BEAT__BEAT_PREDICTOR_HPP