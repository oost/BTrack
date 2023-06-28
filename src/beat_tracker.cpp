//=======================================================================
/** @file BTrack.cpp
 *  @brief BTrack - a real-time beat tracker
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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numbers>
#include <numeric>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/transform.hpp>
#include <ranges>
#include <samplerate.h>
#include <spdlog/spdlog.h>

#include "beat_tracker.hpp"
#include "transformers/beat/all.h"
#include "transformers/manipulation/all.h"
#include "utils.h"

namespace btrack {

using transformers::AdaptiveThreshold;
using transformers::BalancedACF;
using transformers::BeatPredictor;
using transformers::Buffer;
using transformers::CombFilterBank;
using transformers::ExtendTransformer;
using transformers::Resampler;
using transformers::ScoreAccumulator;
using transformers::TempoCalculator;

//=======================================================================
// BTrack::BTrack() : BTrack(512, 1024, 44100) {}

//=======================================================================
BTrack::BTrack(std::size_t hop_size) : BTrack(hop_size, 2 * hop_size, 44100) {}

//=======================================================================
BTrack::BTrack(std::size_t hop_size, std::size_t frame_size,
               double sampling_rate)
    : sampling_rate_{sampling_rate} {
  // initialise parameters
  tightness_ = 5;
  alpha_ = 0.9;
  tempo_ = 120;

  beat_due_in_frame_ = false;
  // tempo is not fixed
  tempo_fixed_ = false;

  odf_ = std::make_unique<OnsetDetectionFunction>(
      hop_size, frame_size, DetectionFunctionType::ComplexSpectralDifferenceHWR,
      WindowType::HanningWindow);
  odf_input_buffer_ = std::make_shared<RealArrayBuffer>(hop_size);
  odf_->set_input(odf_input_buffer_);
  odf_output_ = odf_->output_cast<SingleValueBuffer<double>>();

  recent_beat_weights_ = std::vector<double>(recent_beats_len_);
  for (int i = 0; i < recent_beats_len_; i++) {
    recent_beat_weights_[i] = std::exp(
        -static_cast<double>(i - static_cast<int>(recent_beats_len_) + 1) /
        static_cast<double>(beat_half_life_) * std::log(0.5));
  }
  set_hop_size(hop_size);
}

void BTrack::initialize_state_variables() {
  hop_counter_ = 0;
  last_beat_time_point_ = 0;

  onset_samples_ =
      std::make_shared<CircularBuffer<double>>(onset_df_buffer_len_);
  recent_beat_periods_ =
      std::make_shared<CircularBuffer<double>>(recent_beats_len_);

  m0_ptr = std::make_shared<SingleValueBuffer<int>>();
  beat_counter_ptr = std::make_shared<SingleValueBuffer<int>>();
  beat_period_ptr = std::make_shared<SingleValueBuffer<double>>();
  odf_sample_ptr = std::make_shared<SingleValueBuffer<double>>();
  estimated_tempo_ptr = std::make_shared<SingleValueBuffer<double>>(120.0);

  // Set initial values for m0 and beat_counter
  m0_ptr->set_value(10);
  beat_counter_ptr->set_value(-1);
  beat_period_ptr->set_value(beat_period_from_tempo(tempo_));

  for (int i = 0; i < recent_beat_periods_->size(); i++) {
    recent_beat_periods_->append(60.0 / estimated_tempo_ptr->value());
  }

  // initialise df_buffer to zeros
  int period = std::round(beat_period_ptr->value());
  for (int i = 0; i < onset_df_buffer_len_; i++) {
    (*onset_samples_)[i] = 0;

    if ((i % period) == 0) {
      (*onset_samples_)[i] = 1;
    }
  }
}

void BTrack::create_pipelines() {
  create_score_accumulator_pipeline();
  create_beat_predictor_pipeline();
  create_tempo_calculator_pipeline();
}

/**
 * @brief Score accumulator pipeline
 * Creates score accumulator pipeline
 */
void BTrack::create_score_accumulator_pipeline() {
  // Input
  MultiBuffer::Ptr input_buffer_ptr = std::make_shared<MultiBuffer>();
  input_buffer_ptr->add_buffer(transformers::constants::beat_period_id,
                               beat_period_ptr);
  input_buffer_ptr->add_buffer(transformers::constants::odf_sample_id,
                               odf_sample_ptr);

  // Create score accumulator
  std::shared_ptr<ScoreAccumulator> score_accumulator =
      std::make_shared<ScoreAccumulator>(tightness_, alpha_,
                                         onset_df_buffer_len_);

  // Beat Predictor
  score_accumulator_pipeline_ = std::make_shared<TransformerPipeline>();
  score_accumulator_pipeline_ >> score_accumulator;
  input_buffer_ptr >> score_accumulator_pipeline_;

  cumulative_score_ptr =
      score_accumulator_pipeline_->output_cast<CircularBuffer<double>>();
}

/**
 * @brief Beat predictor pipeline
 *
 */
void BTrack::create_beat_predictor_pipeline() {
  // Input
  MultiBuffer::Ptr input_buffer_ptr = std::make_shared<MultiBuffer>();
  input_buffer_ptr->add_buffer(transformers::constants::beat_counter_id,
                               beat_counter_ptr);
  input_buffer_ptr->add_buffer(transformers::constants::beat_period_id,
                               beat_period_ptr);
  input_buffer_ptr->add_buffer(transformers::constants::cumulative_score_id,
                               cumulative_score_ptr);

  // Create beat predictor
  std::shared_ptr<BeatPredictor> beat_predictor =
      std::make_shared<BeatPredictor>(tightness_);

  // Beat Predictor
  beat_predictor_pipeline_ = std::make_shared<TransformerPipeline>();
  beat_predictor_pipeline_ >> beat_predictor;
  input_buffer_ptr >> beat_predictor_pipeline_;

  MultiBuffer::Ptr out = beat_predictor_pipeline_->output_cast<MultiBuffer>();

  beat_counter_out = out->buffer_cast<SingleValueBuffer<int>>(
      transformers::constants::beat_counter_id);
  m0_out =
      out->buffer_cast<SingleValueBuffer<int>>(transformers::constants::m0_id);
}

/**
 * @brief Tempo calculator pipeline
 *
 */
void BTrack::create_tempo_calculator_pipeline() {

  int resample_len = 512;
  int fft_len = 1024;
  int comb_filter_len_ = 128;
  Transformer::Ptr resampler_ = std::make_shared<Resampler>(resample_len);

  Transformer::Ptr adaptive_threshold_onset_ =
      std::make_shared<AdaptiveThreshold>(resample_len);

  Transformer::Ptr extender =
      std::make_shared<ExtendTransformer<double, double>>(fft_len);

  Transformer::Ptr balanced_acf_ = std::make_shared<BalancedACF>(fft_len);

  Transformer::Ptr comb_filter_bank_ =
      std::make_shared<CombFilterBank>(comb_filter_len_);

  Transformer::Ptr adaptive_threshold_acf_ =
      std::make_shared<AdaptiveThreshold>(comb_filter_len_);

  tempo_calculator_ = std::make_shared<TempoCalculator>(
      sampling_rate_, hop_size_, tempo_fixed_);

  tempo_calculator_pipeline_ = std::make_shared<TransformerPipeline>();

  tempo_calculator_pipeline_ >> resampler_ >> adaptive_threshold_onset_ >>
      extender >> balanced_acf_ >> comb_filter_bank_ >>
      adaptive_threshold_acf_ >> tempo_calculator_;

  onset_samples_ >> tempo_calculator_pipeline_;

  MultiBuffer::Ptr out = tempo_calculator_pipeline_->output_cast<MultiBuffer>();

  beat_period_out = out->buffer_cast<SingleValueBuffer<double>>(
      transformers::constants::beat_period_id);
  estimated_tempo_out = out->buffer_cast<SingleValueBuffer<double>>(
      transformers::constants::estimated_tempo_id);
}
/**
 * @brief
 *
 * @param frame_number
 * @param hop_size
 * @param sampling_frequency
 * @return double
 */
double BTrack::get_beat_time_in_seconds(long frame_number) const {
  return static_cast<double>(frame_number) * static_cast<double>(hop_size_) /
         sampling_rate_;
}

double BTrack::get_current_beat_time_in_seconds() const {
  return get_beat_time_in_seconds(hop_counter_);
}

/**
 * @brief
 *
 *
 * @param frame_number
 * @param hop_size
 * @param sampling_frequency
 * @return
 */

double BTrack::get_beat_time_in_seconds(int frame_number) const {
  return get_beat_time_in_seconds(static_cast<long>(frame_number));
}

void BTrack::set_hop_size(std::size_t hop_size) {
  hop_size_ = hop_size;
  onset_df_buffer_len_ = (512 * 512) / hop_size; // calculate df buffer size

  initialize_state_variables();
  create_pipelines();
}

//=======================================================================
bool BTrack::beat_due_in_current_frame() const { return beat_due_in_frame_; }

//=======================================================================
double BTrack::get_current_tempo_estimate() const {
  return estimated_tempo_ptr->value();
}

double BTrack::recent_average_tempo() const {
  using namespace ranges;
  double sum = 0;
  double sum_w = 0;
  for (int i = 0; i < recent_beat_periods_->size(); i++) {
    sum += recent_beat_periods_->operator[](i) * recent_beat_weights_[i];
    sum_w += recent_beat_weights_[i];
  }

  return 60.0 * sum_w / sum;
}

//=======================================================================
int BTrack::get_hop_size() const { return hop_size_; }

void BTrack::process_audio_frame(std::span<double> frame) {
  std::copy(frame.begin(), frame.end(), odf_input_buffer_->data().begin());

  // calculate the onset detection function sample for the frame
  odf_->execute();
  double sample = odf_output_->value();

  // process the new onset detection function sample in the beat tracking
  // algorithm
  process_onset_detection_function_sample(sample);
}

//=======================================================================
void BTrack::process_audio_frame(std::vector<double> &frame) {
  std::span<double> input_span = std::span(frame);
  process_audio_frame(input_span);
}

// void BTrack::process_audio_frame(RealArrayBuffer::Ptr input_buffer) {
//   // calculate the onset detection function sample for the frame
//   odf_->set_input(input_buffer);
//   odf_->execute();
//   double sample = odf_output_->value();

//   // process the new onset detection function sample in the beat tracking
//   // algorithm
//   process_onset_detection_function_sample(sample);
// }

// void BTrackLegacyAdapter::processAudioFrame(double *frame) {
//   std::vector<double> frame_vec(frame, frame + hop_size_);
//   process_audio_frame(frame_vec);
// }

//=======================================================================
void BTrack::process_onset_detection_function_sample(double new_sample) {
  hop_counter_++;

  // we need to ensure that the onset
  // detection function sample is positive
  new_sample = fabs(new_sample);

  // add a tiny constant to the sample to stop it from ever going
  // to zero. this is to avoid problems further down the line
  new_sample = new_sample + 0.0001;

  m0_ptr->decrement(1);
  beat_counter_ptr->decrement(1);
  beat_due_in_frame_ = false;

  // add new sample at the end
  onset_samples_->append(new_sample);
  odf_sample_ptr->set_value(new_sample);
  /**
   *
   * m0:
   *   - ?
   *
   * beat_counter:
   *   - backward counter to next expected beat. At 0, beat is expected
   *   - updated by BeatPredictor
   *
   * beat_period:
   *   - Beat period in # of hops:
   *   - beatPeriod = round(60 / ((((double)hopSize) / sampling_rate) *
   * tempo));
   *   - Updated by TempoCalculator.
   *
   * estimated_tempo
   *   - beat_period expressed as a frequency
   *
   * Pipeline:
   *  Score Accumulator pipeline
   *      1 - Score Accumulator
   *          (beat_period, onset_samples_) => cumulative_score
   *
   *  Beat Predictor Pipeline:
   *    - Updates m0 and beat_counter...
   *    - Steps
   *      1 - on m0 == 0: Beat Predictor
   *          (m0, beat_period, cumulative_score) => (m0, beat_counter)
   *
   *  Tempo Calculation
   *    - on beat_counter == 0 (i.e. only once every extimated beats)
   *    - update beat_period and estimated_tempo
   *    - Steps:
   *      1 - Resample
   *          (onset_samples_) => (resampled_onset_samples_[512])
   *      2 - adaptiveThreshold
   *          (resampled_onset_samples_[512]) =>
   * (onset_threshold_filtered[512]) 3 - calculateBalancedACF
   *          (onset_threshold_filtered[512]) => (acf[128])
   *      4 - calculateOutputOfCombFilterBank
   *          (acf[128]) => (combFilterBankOutput[128])
   *      5 - adaptiveThreshold
   *          (combFilterBankOutput[128]) => (comb_threshold_filtered[128])
   *      6 - Calculate tempo
   *          (comb_threshold_filtered[128]) => (beat_period_,
   * estimated_tempo_)
   */

  score_accumulator_pipeline_->execute();
  if (m0_ptr->value() == 0) {
    beat_predictor_pipeline_->execute();
    // Reset beat counter for next beat
    beat_counter_ptr->set_value(beat_counter_out->value());

    // Reset m0 counter for next prediction
    m0_ptr->set_value(m0_out->value());

    // Call next beat callback
    if (on_next_beat_cb_) {
      auto samples_to_next_beat = beat_counter_ptr->value();
      on_next_beat_cb_((samples_to_next_beat * hop_size_ * 1000000) /
                           sampling_rate_,
                       get_current_tempo_estimate(), recent_average_tempo());

      SPDLOG_INFO(
          "Prediction beat_counter_ptr {}, time {}", samples_to_next_beat,
          (samples_to_next_beat * hop_size_ * 1000000) / sampling_rate_);
    }
  }

  // if we are at a beat
  if (beat_counter_ptr->value() == 0) {
    // indicate a beat should be output
    beat_due_in_frame_ = true;

    double current_beat_time_point_ = get_current_beat_time_in_seconds();
    recent_beat_periods_->append(current_beat_time_point_ -
                                 last_beat_time_point_);
    last_beat_time_point_ = current_beat_time_point_;

    tempo_calculator_pipeline_->execute();
    beat_period_ptr->set_value(beat_period_out->value());
    estimated_tempo_ptr->set_value(estimated_tempo_out->value());
    if (on_beat_cb_) {
      on_beat_cb_(get_current_tempo_estimate(), recent_average_tempo());
    }
  }
}

int BTrack::beat_period_from_tempo(double tempo) {
  return static_cast<int>(
      round(60.0 / tempo * (sampling_rate_ / static_cast<double>(hop_size_))));
}

// void BTrackLegacyAdapter::processOnsetDetectionFunctionSample(
//     double newSample) {
//   process_onset_detection_function_sample(newSample);
// }

//=======================================================================
void BTrack::set_tempo(double tempo) {

  /////////// TEMPO INDICATION RESET //////////////////

  // firstly make sure tempo is between 80 and 160 bpm..
  while (tempo > 160) {
    tempo = tempo / 2;
  }

  while (tempo < 80) {
    tempo = tempo * 2;
  }

  std::dynamic_pointer_cast<TempoCalculator>(tempo_calculator_)
      ->reset_delta(tempo);

  /////////// CUMULATIVE SCORE ARTIFICAL TEMPO UPDATE //////////////////

  // calculate new beat period
  int new_bperiod = beat_period_from_tempo(tempo);

  int bcounter = 1;
  // initialise df_buffer to zeros
  for (int i = (onset_df_buffer_len_ - 1); i >= 0; i--) {
    if (bcounter == 1) {
      // (*cumulative_score_ptr)[i] = 150;
      (*onset_samples_)[i] = 150;
    } else {
      // (*cumulative_score_ptr)[i] = 10;
      (*onset_samples_)[i] = 10;
    }

    bcounter++;

    if (bcounter > new_bperiod) {
      bcounter = 1;
    }
  }

  /////////// INDICATE THAT THIS IS A BEAT //////////////////

  // beat is now
  beat_counter_ptr->set_value(0);

  // offbeat is half of new beat period away
  m0_ptr->set_value(round(static_cast<double>(new_bperiod) / 2));
}

//=======================================================================
void BTrack::fix_tempo(double tempo) {
  // firstly make sure tempo is between 80 and 160 bpm..
  while (tempo > 160) {
    tempo = tempo / 2;
  }

  while (tempo < 80) {
    tempo = tempo * 2;
  }

  std::dynamic_pointer_cast<TempoCalculator>(tempo_calculator_)
      ->reset_delta(tempo);

  // set the tempo fix flag
  tempo_fixed_ = true;
}

//=======================================================================
void BTrack::do_not_fix_tempo() {
  // set the tempo fix flag
  tempo_fixed_ = false;
}

} // namespace btrack