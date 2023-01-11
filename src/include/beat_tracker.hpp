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

#include "circular_buffer.h"
#include "onset_detection_function.hpp"
#include "transformers/fft_operator.hpp"
#include "transformers/transformer_pipeline.hpp"
#include <vector>

using transformers::ComplexDataBuffer;
using transformers::DataBuffer;
using transformers::FFTOperator;
using transformers::MultiBuffer;

// #include "btrack_config.h"

//=======================================================================
/** The main beat tracking class and the interface to the BTrack
 * beat tracking algorithm. The algorithm can process either
 * audio frames or onset detection function samples and also
 * contains some static functions for calculating beat times in seconds
 */

class BTrack {

public:
  //=======================================================================
  /** Constructor assuming hop size of 512 and frame size of 1024 */
  BTrack();

  /** Constructor assuming frame size will be double the hopSize
   * @param hop_size the hop size in audio samples
   */
  BTrack(int hop_size);

  /** Constructor taking both hop_size and frame_size
   * @param hop_size the hop size in audio samples
   * @param frame_size the frame size in audio samples
   */
  BTrack(int hop_size, int frame_size, int sampling_rate = 44100);

  /** Destructor */
  ~BTrack();

  //=======================================================================
  /** Updates the hop and frame size used by the beat tracker
   * @param hop_size the hop size in audio samples
   * @param frame_size the frame size in audio samples
   */
  void updateHopAndFrameSize(int hop_size, int frame_size);

  //=======================================================================
  /** Process a single audio frame
   * @param frame a pointer to an array containing an audio frame. The number of
   * samples should match the frame size that the algorithm was initialised
   * with.
   */
  void process_audio_frame(std::vector<double> &frame);
  void processAudioFrame(double *frame);

  /** Add new onset detection function sample to buffer and apply beat tracking
   * @param sample an onset detection function sample
   */
  void processOnsetDetectionFunctionSample(double sample);

  //=======================================================================
  /** @returns the current hop size being used by the beat tracker */
  int get_hop_size();

  /** @returns true if a beat should occur in the current audio frame */
  bool beatDueInCurrentFrame();

  /** @returns the current tempo estimate being used by the beat tracker */
  double getCurrentTempoEstimate();

  /** @returns the most recent value of the cumulative score function */
  double getLatestCumulativeScoreValue();

  //=======================================================================
  /** Set the tempo of the beat tracker
   * @param tempo the tempo in beats per minute (bpm)
   */
  void setTempo(double tempo);

  /** Fix tempo to roughly around some value, so that the algorithm will only
   * try to track tempi around the given tempo
   * @param tempo the tempo in beats per minute (bpm)
   */
  void fixTempo(double tempo);

  /** Tell the algorithm to not fix the tempo anymore */
  void doNotFixTempo();

  //=======================================================================
  /** Calculates a beat time in seconds, given the frame number, hop size and
   * sampling frequency. This version uses a long to represent the frame number
   * @param frameNumber the index of the current frame
   * @param hop_size the hop size in audio samples
   * @param fs the sampling frequency in Hz
   * @returns a beat time in seconds
   */
  static double getBeatTimeInSeconds(long frameNumber, int hop_size, int fs);

  /** Calculates a beat time in seconds, given the frame number, hop size and
   * sampling frequency. This version uses an int to represent the frame number
   * @param frameNumber the index of the current frame
   * @param hop_size the hop size in audio samples
   * @param fs the sampling frequency in Hz
   * @returns a beat time in seconds
   */
  static double getBeatTimeInSeconds(int frameNumber, int hop_size, int fs);

private:
  /** Initialise with hop size and set all array sizes accordingly
   * @param hop_size_ the hop size in audio samples
   */
  void set_hop_size(int hop_size);

  /** Resamples the onset detection function from an arbitrary number of samples
   * to 512 */
  void resampleOnsetDetectionFunction();

  /** Updates the cumulative score function with a new onset detection function
   * sample
   * @param odfSample an onset detection function sample
   */
  void updateCumulativeScore(double odfSample);

  /** Predicts the next beat, based upon the internal program state */
  void predictBeat();

  /** Calculates the current tempo expressed as the beat period in detection
   * function samples */
  void calculateTempo();

  /** Calculates an adaptive threshold which is used to remove low level energy
   * from detection function and emphasise peaks
   * @param x a pointer to an array containing onset detection function samples
   * @param N the length of the array, x
   */
  void adaptiveThreshold(std::vector<double> &x);

  /** Calculates the balanced autocorrelation of the smoothed onset detection
   * function
   * @param onsetDetectionFunction a pointer to an array containing the onset
   * detection function
   */
  void calculateBalancedACF(const std::vector<double> &onsetDetectionFunction);

  /** Calculates the output of the comb filter bank */
  void calculateOutputOfCombFilterBank();

  //=======================================================================

  /** An OnsetDetectionFunction instance for calculating onset detection
   * functions */
  OnsetDetectionFunction::Ptr odf_;

  //=======================================================================
  // buffers

  CircularBuffer<double> onsetDF_; /**< to hold onset detection function */
  CircularBuffer<double> cumulativeScore_; /**< to hold cumulative score */

  std::vector<double>
      resampledOnsetDF_;    /**< to hold resampled detection function */
  std::vector<double> acf_; /**<  to hold autocorrelation function */
  std::vector<double> weightingVector_;      /**<  to hold weighting vector */
  std::vector<double> combFilterBankOutput_; /**<  to hold comb filter output */
  double tempoObservationVector_[41]; /**<  to hold tempo version of comb filter
                                        output */
  std::vector<double> delta_;     /**<  to hold final tempo candidate array */
  std::vector<double> prevDelta_; /**<  previous delta */
  std::vector<double>
      prevDeltaFixed_; /**<  fixed tempo version of previous delta */
  double tempoTransitionMatrix_[41][41]; /**<  tempo transition matrix */

  //=======================================================================
  // parameters

  double tightness_; /**< the tightness of the weighting used to calculate
                       cumulative score */
  double alpha_; /**< the mix between the current detection function sample and
                   the cumulative score's "momentum" */
  double beatPeriod_;     /**< the beat period, in detection function samples */
  double tempo_;          /**< the tempo in beats per minute */
  double estimatedTempo_; /**< the current tempo estimation being used by the
                            algorithm */
  double latestCumulativeScoreValue_; /**< holds the latest value of the
                                        cumulative score function */
  double tempoToLagFactor_; /**< factor for converting between lag and tempo */
  int m0_; /**< indicates when the next point to predict the next beat is */
  int beatCounter_; /**< keeps track of when the next beat is - will be zero
                      when the beat is due, and is set elsewhere in the
                      algorithm to be positive once a beat prediction is made */
  int hop_size_;    /**< the hop size being used by the algorithm */
  int onsetDFBufferSize_; /**< the onset detection function buffer size */
  bool tempoFixed_; /**< indicates whether the tempo should be fixed or not */
  bool
      beatDueInFrame_; /**< indicates whether a beat is due in the current frame
                        */
  int FFTLengthForACFCalculation_; /**< the FFT length for the auto-correlation
                                     function calculation */
  FFTOperator::Ptr fft_operator_;
  FFTOperator::Ptr fft_operator_backwards_;
  ComplexDataBuffer::Ptr fft_input_buffer_;

  TransformerPipeline<MultiBuffer>::Ptr pipeline_;

  int sampling_rate_;
};

#endif
