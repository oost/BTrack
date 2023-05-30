//=======================================================================
/** @file beat_tracker.hpp
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

#ifndef __BTRACK_H
#define __BTRACK_H

#include <functional>
#include <span>
#include <vector>

#include "onset_detection_function.h"
#include "transformers/buffers/all.h"
#include "transformers/fft_operator.hpp"
#include "transformers/transformer_pipeline.hpp"
#include "transformers/transformers/all.h"

// #include "btrack_config.h"

//=======================================================================
/** The main beat tracking class and the interface to the BTrack
 * beat tracking algorithm. The algorithm can process either
 * audio frames or onset detection function samples and also
 * contains some static functions for calculating beat times in seconds
 */

namespace btrack {

using transformers::ArrayBuffer;
using transformers::CircularBuffer;
using transformers::ComplexArrayBuffer;
using transformers::FFTOperator;
using transformers::MultiBuffer;
using transformers::RealArrayBuffer;
using transformers::SingleValueBuffer;
using transformers::Transformer;

class BTrack {

public:
  using Ptr = std::unique_ptr<BTrack>;
  using beat_cb_t = std::function<void(double, double)>;
  using next_beat_cb_t = std::function<void(uint64_t, double, double)>;

  /** Constructor assuming frame size will be double the hopSize
   * @param hop_size the hop size in audio samples
   */
  BTrack(std::size_t hop_size = 512);

  /** Constructor taking both hop_size and frame_size
   * @param hop_size the hop size in audio samples
   * @param frame_size the frame size in audio samples
   */
  BTrack(std::size_t hop_size, std::size_t frame_size,
         double sampling_rate = 44100);

  /** Destructor */
  ~BTrack() {}

  void set_sampling_rate(double sampling_rate) {
    sampling_rate_ = sampling_rate;
    // Recreate tempo calculator pipeline (as last step has a dependency on the
    // sampling rate)
    create_tempo_calculator_pipeline();
  }

  void set_beat_callback(beat_cb_t beat_cb) { on_beat_cb_ = beat_cb; }
  void set_next_beat_callback(next_beat_cb_t next_beat_cb) {
    on_next_beat_cb_ = next_beat_cb;
  }

  //=======================================================================
  /** Process a single audio frame
   * @param frame a pointer to an array containing an audio frame. The number of
   * samples should match the frame size that the algorithm was initialised
   * with.
   */
  void process_audio_frame(std::vector<double> &frame);
  void process_audio_frame(std::span<double> frame);
  // void process_audio_frame(RealArrayBuffer::Ptr input_buffer);

  //=======================================================================
  /** @returns the current hop size being used by the beat tracker */
  int get_hop_size() const;

  /** @returns true if a beat should occur in the current audio frame */
  bool beat_due_in_current_frame() const;

  /** @returns the current tempo estimate being used by the beat tracker */
  double get_current_tempo_estimate() const;

  //=======================================================================
  /** Set the tempo of the beat tracker
   * @param tempo the tempo in beats per minute (bpm)
   */
  void set_tempo(double tempo);

  /** Fix tempo to roughly around some value, so that the algorithm will only
   * try to track tempi around the given tempo
   * @param tempo the tempo in beats per minute (bpm)
   */
  void fix_tempo(double tempo);

  /** Tell the algorithm to not fix the tempo anymore */
  void do_not_fix_tempo();

  //=======================================================================
  /** Calculates a beat time in seconds, given the frame number, hop size and
   * sampling frequency. This version uses a long to represent the frame number
   * @param frame_number the index of the current frame
   * @param hop_size the hop size in audio samples
   * @param fs the sampling frequency in Hz
   * @returns a beat time in seconds
   */
  double get_beat_time_in_seconds(long frame_number) const;

  /** Calculates a beat time in seconds, given the frame number, hop size and
   * sampling frequency. This version uses an int to represent the frame number
   * @param frame_number the index of the current frame
   * @param hop_size the hop size in audio samples
   * @param fs the sampling frequency in Hz
   * @returns a beat time in seconds
   */
  double get_beat_time_in_seconds(int frame_number) const;

  double get_current_beat_time_in_seconds() const;

  double recent_average_tempo() const;

  /** Add new onset detection function sample to buffer and apply beat tracking
   * @param sample an onset detection function sample
   */
  void process_onset_detection_function_sample(double sample);

protected:
  /**
   * @brief
   */
  void initialize_state_variables();

  /**
   * @brief
   *
   */
  void create_beat_predictor_pipeline();
  void create_tempo_calculator_pipeline();
  void create_score_accumulator_pipeline();

  void create_pipelines();

  /**
   * @brief Set the hop size object
   * Initialise with hop size and set all array sizes accordingly
   * @param hop_size the hop size in audio samples
   */

  void set_hop_size(std::size_t hop_size);

  int beat_period_from_tempo(double tempo);

  //=======================================================================

  /** An OnsetDetectionFunction instance for calculating onset detection
   * functions */
  OnsetDetectionFunction::Ptr odf_;
  SingleValueBuffer<double>::Ptr odf_output_;
  RealArrayBuffer::Ptr odf_input_buffer_;

  //=======================================================================
  // parameters

  int hop_counter_ = 0;

  /**< the tightness of the weighting used to calculate cumulative score */
  double tightness_;
  /**< the mix between the current detection function sample and the cumulative
   * score's "momentum" */
  double alpha_;

  double beat_period_;
  /**< the tempo in beats per minute */
  double tempo_;
  double sampling_rate_;
  int beat_half_life_ = 15;

  /**< the hop size being used by the algorithm */
  std::size_t hop_size_;

  /**< the onset detection function buffer size */
  std::size_t onset_df_buffer_len_;

  std::size_t recent_beats_len_ = 30;

  /**< indicates whether the tempo should be fixed or not */
  bool tempo_fixed_;

  /**< indicates whether a beat is due in the current frame */
  bool beat_due_in_frame_;

  Transformer::Ptr tempo_calculator_;

  // Pipelines
  TransformerPipeline::Ptr beat_predictor_pipeline_;
  TransformerPipeline::Ptr tempo_calculator_pipeline_;
  TransformerPipeline::Ptr score_accumulator_pipeline_;

  /**
   * State variables
   *
   */

  /** the beat period, in detection function samples */
  SingleValueBuffer<double>::Ptr beat_period_ptr;
  // CircularBuffer<double>::Ptr cumulative_score_ptr;
  /**< indicates when the next point to predict the next beat is */
  SingleValueBuffer<int>::Ptr m0_ptr;
  /**< keeps track of when the next beat is - will be zero when the beat is due,
   * and is set elsewhere in the algorithm to be positive once a beat prediction
   * is made */
  SingleValueBuffer<int>::Ptr beat_counter_ptr;

  SingleValueBuffer<double>::Ptr odf_sample_ptr;

  /**< the current tempo estimation being used by the algorithm */
  SingleValueBuffer<double>::Ptr estimated_tempo_ptr;
  MultiBuffer::Ptr input_buffer_ptr;

  /**
   * Outputs
   *
   */
  SingleValueBuffer<int>::Ptr beat_counter_out;
  SingleValueBuffer<int>::Ptr m0_out;
  SingleValueBuffer<double>::Ptr beat_period_out;
  SingleValueBuffer<double>::Ptr estimated_tempo_out;

  /**< to hold onset detection function */
  CircularBuffer<double>::Ptr onset_samples_;
  /**< to hold cumulative score */
  CircularBuffer<double>::Ptr cumulative_score_ptr;

  /**< to hold recent beats*/
  CircularBuffer<double>::Ptr recent_beat_periods_;
  std::vector<double> recent_beat_weights_;
  double last_beat_time_point_ = 0;

  beat_cb_t on_beat_cb_;
  next_beat_cb_t on_next_beat_cb_;
};

} // namespace btrack
#endif
