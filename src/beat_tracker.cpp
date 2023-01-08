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
#include <cmath>
#include <iostream>
#include <numbers>
#include <numeric>
#include <ranges>
#include <samplerate.h>

#include "beat_tracker.hpp"
#include "utils.hpp"

constexpr double rayparam = 43.0;
constexpr std::size_t combfilterbank_size_ = 128;

//=======================================================================
BTrack::BTrack() : BTrack(512, 1024, 44100) {}

//=======================================================================
BTrack::BTrack(int hopSize) : BTrack(hopSize, 2 * hopSize, 44100) {}

//=======================================================================
BTrack::BTrack(int hopSize, int frameSize, int sampling_rate)
    : sampling_rate_{sampling_rate} {

  weightingVector_.resize(combfilterbank_size_);
  combFilterBankOutput_.resize(combfilterbank_size_);
  resampledOnsetDF_.resize(512);
  acf_.resize(512);
  delta_.resize(41);
  prevDelta_.resize(41);
  prevDeltaFixed_.resize(41);

  odf_ = std::make_unique<OnsetDetectionFunction>(
      hopSize, frameSize, DetectionFunctionType::ComplexSpectralDifferenceHWR,
      WindowType::HanningWindow);

  // initialise parameters
  tightness_ = 5;
  alpha_ = 0.9;
  tempo_ = 120;
  estimatedTempo_ = 120.0;
  tempoToLagFactor_ = 60.0 * sampling_rate_ / 512.;

  m0_ = 10;
  beatCounter_ = -1;

  beatDueInFrame_ = false;

  // create rayleigh weighting vector
  for (int n = 0; n < weightingVector_.size(); n++) {
    weightingVector_[n] =
        ((double)n / pow(rayparam, 2)) *
        exp((-1 * pow((double)-n, 2)) / (2 * pow(rayparam, 2)));
  }

  // initialise prev_delta
  for (int i = 0; i < 41; i++) {
    prevDelta_[i] = 1;
  }

  double t_mu = 41 / 2;
  double m_sig;
  double x;
  // create tempo transition matrix
  m_sig = 41 / 8;
  for (int i = 0; i < 41; i++) {
    for (int j = 0; j < 41; j++) {
      x = j + 1;
      t_mu = i + 1;
      tempoTransitionMatrix_[i][j] =
          (1 / (m_sig * sqrt(2 * std::numbers::pi))) *
          exp((-1 * pow((x - t_mu), 2)) / (2 * pow(m_sig, 2)));
    }
  }

  // tempo is not fixed
  tempoFixed_ = false;

  // initialise latest cumulative score value
  // in case it is requested before any processing takes place
  latestCumulativeScoreValue_ = 0;

  // initialise algorithm given the hopsize
  setHopSize(hopSize);

  // Set up FFT for calculating the auto-correlation function
  FFTLengthForACFCalculation_ = 1024;

  // fft_operator_ =
  //     FFTOperator::createOperator(FFTLengthForACFCalculation_, true);
}

//=======================================================================
BTrack::~BTrack() {
  // fft_operator_ = nullptr;
}

//=======================================================================
double BTrack::getBeatTimeInSeconds(long frameNumber, int hopSize, int fs) {
  double hop = (double)hopSize;
  double samplingFrequency = (double)fs;
  double frameNum = (double)frameNumber;

  return ((hop / samplingFrequency) * frameNum);
}

//=======================================================================
double BTrack::getBeatTimeInSeconds(int frameNumber, int hopSize, int fs) {
  long frameNum = (long)frameNumber;

  return getBeatTimeInSeconds(frameNum, hopSize, fs);
}

//=======================================================================
void BTrack::setHopSize(int hopSize) {
  hopSize_ = hopSize;
  onsetDFBufferSize_ = (512 * 512) / hopSize; // calculate df buffer size

  beatPeriod_ = round(60 / ((((double)hopSize) / sampling_rate_) * tempo_));

  // set size of onset detection function buffer
  onsetDF_.resize(onsetDFBufferSize_);

  // set size of cumulative score buffer
  cumulativeScore_.resize(onsetDFBufferSize_);

  // initialise df_buffer to zeros
  for (int i = 0; i < onsetDFBufferSize_; i++) {
    onsetDF_[i] = 0;
    cumulativeScore_[i] = 0;

    if ((i % ((int)round(beatPeriod_))) == 0) {
      onsetDF_[i] = 1;
    }
  }
}

//=======================================================================
void BTrack::updateHopAndFrameSize(int hopSize, int frameSize) {
  // update the onset detection function object
  odf_ = std::make_unique<OnsetDetectionFunction>(hopSize, frameSize);

  // update the hop size being used by the beat tracker
  setHopSize(hopSize);
}

//=======================================================================
bool BTrack::beatDueInCurrentFrame() { return beatDueInFrame_; }

//=======================================================================
double BTrack::getCurrentTempoEstimate() { return estimatedTempo_; }

//=======================================================================
int BTrack::getHopSize() { return hopSize_; }

//=======================================================================
double BTrack::getLatestCumulativeScoreValue() {
  return latestCumulativeScoreValue_;
}

//=======================================================================
void BTrack::processAudioFrame(std::vector<double> &frame) {
  // calculate the onset detection function sample for the frame
  double sample = odf_->calculateOnsetDetectionFunctionSample(frame);

  // process the new onset detection function sample in the beat tracking
  // algorithm
  processOnsetDetectionFunctionSample(sample);
}

void BTrack::processAudioFrame(double *frame) {
  std::vector<double> frame_vec(frame, frame + hopSize_);
}

//=======================================================================
void BTrack::processOnsetDetectionFunctionSample(double newSample) {
  // we need to ensure that the onset
  // detection function sample is positive
  newSample = fabs(newSample);

  // add a tiny constant to the sample to stop it from ever going
  // to zero. this is to avoid problems further down the line
  newSample = newSample + 0.0001;

  m0_--;
  beatCounter_--;
  beatDueInFrame_ = false;

  // add new sample at the end
  onsetDF_.addSampleToEnd(newSample);

  // update cumulative score
  updateCumulativeScore(newSample);

  // if we are halfway between beats
  if (m0_ == 0) {
    predictBeat();
  }

  // if we are at a beat
  if (beatCounter_ == 0) {
    beatDueInFrame_ = true; // indicate a beat should be output

    // recalculate the tempo
    resampleOnsetDetectionFunction();
    calculateTempo();
  }
}

//=======================================================================
void BTrack::setTempo(double tempo) {

  /////////// TEMPO INDICATION RESET //////////////////

  // firstly make sure tempo is between 80 and 160 bpm..
  while (tempo > 160) {
    tempo = tempo / 2;
  }

  while (tempo < 80) {
    tempo = tempo * 2;
  }

  // convert tempo from bpm value to integer index of tempo probability
  int tempo_index = (int)round((tempo - 80) / 2);

  // now set previous tempo observations to zero
  for (int i = 0; i < 41; i++) {
    prevDelta_[i] = 0;
  }

  // set desired tempo index to 1
  prevDelta_[tempo_index] = 1;

  /////////// CUMULATIVE SCORE ARTIFICAL TEMPO UPDATE //////////////////

  // calculate new beat period
  int new_bperiod = static_cast<int>(
      round(60 / ((static_cast<double>(hopSize_) / sampling_rate_) * tempo)));

  int bcounter = 1;
  // initialise df_buffer to zeros
  for (int i = (onsetDFBufferSize_ - 1); i >= 0; i--) {
    if (bcounter == 1) {
      cumulativeScore_[i] = 150;
      onsetDF_[i] = 150;
    } else {
      cumulativeScore_[i] = 10;
      onsetDF_[i] = 10;
    }

    bcounter++;

    if (bcounter > new_bperiod) {
      bcounter = 1;
    }
  }

  /////////// INDICATE THAT THIS IS A BEAT //////////////////

  // beat is now
  beatCounter_ = 0;

  // offbeat is half of new beat period away
  m0_ = (int)round(((double)new_bperiod) / 2);
}

//=======================================================================
void BTrack::fixTempo(double tempo) {
  // firstly make sure tempo is between 80 and 160 bpm..
  while (tempo > 160) {
    tempo = tempo / 2;
  }

  while (tempo < 80) {
    tempo = tempo * 2;
  }

  // convert tempo from bpm value to integer index of tempo probability
  int tempo_index = (int)round((tempo - 80) / 2);

  // now set previous fixed previous tempo observation values to zero
  for (int i = 0; i < 41; i++) {
    prevDeltaFixed_[i] = 0;
  }

  // set desired tempo index to 1
  prevDeltaFixed_[tempo_index] = 1;

  // set the tempo fix flag
  tempoFixed_ = true;
}

//=======================================================================
void BTrack::doNotFixTempo() {
  // set the tempo fix flag
  tempoFixed_ = false;
}

//=======================================================================
void BTrack::resampleOnsetDetectionFunction() {
  float output[512];

  float input[onsetDFBufferSize_];

  for (int i = 0; i < onsetDFBufferSize_; i++) {
    input[i] = (float)onsetDF_[i];
  }

  double src_ratio = 512.0 / ((double)onsetDFBufferSize_);
  int BUFFER_LEN = onsetDFBufferSize_;
  int output_len;
  SRC_DATA src_data;

  // output_len = (int) floor (((double) BUFFER_LEN) * src_ratio) ;
  output_len = 512;

  src_data.data_in = input;
  src_data.input_frames = BUFFER_LEN;

  src_data.src_ratio = src_ratio;

  src_data.data_out = output;
  src_data.output_frames = output_len;

  src_simple(&src_data, SRC_SINC_BEST_QUALITY, 1);

  for (int i = 0; i < output_len; i++) {
    resampledOnsetDF_[i] = (double)src_data.data_out[i];
  }
}

//=======================================================================
void BTrack::calculateTempo() {
  // adaptive threshold on input
  adaptiveThreshold(resampledOnsetDF_);

  // calculate auto-correlation function of detection function
  calculateBalancedACF(resampledOnsetDF_);

  // calculate output of comb filterbank
  calculateOutputOfCombFilterBank();

  // adaptive threshold on rcf
  adaptiveThreshold(combFilterBankOutput_);

  int t_index;
  int t_index2;
  // calculate tempo observation vector from beat period observation vector
  for (int i = 0; i < 41; i++) {
    t_index = (int)round(tempoToLagFactor_ / ((double)((2 * i) + 80)));
    t_index2 = (int)round(tempoToLagFactor_ / ((double)((4 * i) + 160)));

    tempoObservationVector_[i] = combFilterBankOutput_[t_index - 1] +
                                 combFilterBankOutput_[t_index2 - 1];
  }

  double maxval;
  double maxind;
  double curval;

  // if tempo is fixed then always use a fixed set of tempi as the previous
  // observation probability function
  if (tempoFixed_) {
    for (int k = 0; k < 41; k++) {
      prevDelta_[k] = prevDeltaFixed_[k];
    }
  }

  for (int j = 0; j < 41; j++) {
    maxval = -1;
    for (int i = 0; i < 41; i++) {
      curval = prevDelta_[i] * tempoTransitionMatrix_[i][j];

      if (curval > maxval) {
        maxval = curval;
      }
    }

    delta_[j] = maxval * tempoObservationVector_[j];
  }

  normalizeArray(delta_);

  maxind = -1;
  maxval = -1;

  for (int j = 0; j < 41; j++) {
    if (delta_[j] > maxval) {
      maxval = delta_[j];
      maxind = j;
    }

    prevDelta_[j] = delta_[j];
  }

  beatPeriod_ = round((60.0 * sampling_rate_) /
                      (((2 * maxind) + 80) * ((double)hopSize_)));

  if (beatPeriod_ > 0) {
    estimatedTempo_ =
        60.0 / ((((double)hopSize_) / sampling_rate_) * beatPeriod_);
  }
}

//=======================================================================
void BTrack::adaptiveThreshold(std::vector<double> &x) {
  std::size_t i = 0;
  int k, t = 0;
  double x_thresh[x.size()];

  std::size_t p_post = 7;
  std::size_t p_pre = 8;

  t = std::min(x.size(), p_post); // what is smaller, p_post of df size. This is
                                  // to avoid accessing outside of arrays

  // find threshold for first 't' samples, where a full average cannot be
  // computed yet
  for (i = 0; i <= t; i++) {
    k = std::min((i + p_pre), x.size());
    x_thresh[i] = calculateMeanOfArray(x.begin() + 1, x.begin() + k);
  }
  // find threshold for bulk of samples across a moving average from
  // [i-p_pre,i+p_post]
  for (i = t + 1; i < x.size() - p_post; i++) {
    x_thresh[i] =
        calculateMeanOfArray(x.begin() + i - p_pre, x.begin() + i + p_post);
  }
  // for last few samples calculate threshold, again, not enough samples to do
  // as above
  for (i = x.size() - p_post; i < x.size(); i++) {
    k = std::max((i - p_post), std::size_t(1));
    x_thresh[i] = calculateMeanOfArray(x.begin() + k, x.end());
  }

  // subtract the threshold from the detection function and check that it is not
  // less than 0
  for (i = 0; i < x.size(); i++) {
    x[i] = x[i] - x_thresh[i];
    if (x[i] < 0) {
      x[i] = 0;
    }
  }
}

//=======================================================================
void BTrack::calculateOutputOfCombFilterBank() {
  int numelem;

  for (auto &elem : combFilterBankOutput_) {
    elem = 0;
  }

  numelem = 4;

  for (int i = 2; i <= 127; i++) // max beat period
  {
    for (int a = 1; a <= numelem; a++) // number of comb elements
    {
      for (int b = 1 - a; b <= a - 1;
           b++) // general state using normalisation of comb elements
      {
        combFilterBankOutput_[i - 1] =
            combFilterBankOutput_[i - 1] +
            (acf_[(a * i + b) - 1] * weightingVector_[i - 1]) /
                (2 * a - 1); // calculate value for comb filter row
      }
    }
  }
}

//=======================================================================
void BTrack::calculateBalancedACF(
    const std::vector<double> &onsetDetectionFunction) {
  int onsetDetectionFunctionLength = 512;
  // FFTOperator::complex_v &input = fft_operator_->input();
  // FFTOperator::complex_v &output = fft_operator_->output();

  // // copy into complex array and zero pad
  // for (int i = 0; i < FFTLengthForACFCalculation_; i++) {
  //   if (i < onsetDetectionFunctionLength) {
  //     input[i] = FFTOperator::complex_t(onsetDetectionFunction[i], 0.0);
  //   } else {
  //     input[i] = FFTOperator::complex_t(0, 0.0);
  //   }
  // }
  // fft_operator_->performFFT();

  // // multiply by complex conjugate
  // for (int i = 0; i < FFTLengthForACFCalculation_; i++) {
  //   output[i] = FFTOperator::complex_t(std::norm(output[i]), 0);
  // }

  // fft_operator_->performFFT(true);

  // double lag = 512;

  // for (int i = 0; i < 512; i++) {

  //   // calculate absolute value of result
  //   double absValue = std::abs(fft_operator_->input()[i]);

  //   // divide by inverse lad to deal with scale bias towards small lags
  //   acf_[i] = absValue / lag;

  //   // this division by 1024 is technically unnecessary but it ensures the
  //   // algorithm produces exactly the same ACF output as the old time domain
  //   // implementation. The time difference is minimal so I decided to keep it
  //   acf_[i] = acf_[i] / 1024.;

  //   lag = lag - 1.;
  // }
}

//=======================================================================
void BTrack::updateCumulativeScore(double odfSample) {
  int start, end, winsize;
  double max;

  start = onsetDFBufferSize_ - round(2 * beatPeriod_);
  end = onsetDFBufferSize_ - round(beatPeriod_ / 2);
  winsize = end - start + 1;

  double w1[winsize];
  double v = -2 * beatPeriod_;
  double wcumscore;

  // create window
  for (int i = 0; i < winsize; i++) {
    w1[i] = exp((-1 * pow(tightness_ * log(-v / beatPeriod_), 2)) / 2);
    v = v + 1;
  }

  // calculate new cumulative score value
  max = 0;
  int n = 0;
  for (int i = start; i <= end; i++) {
    wcumscore = cumulativeScore_[i] * w1[n];

    if (wcumscore > max) {
      max = wcumscore;
    }
    n++;
  }

  latestCumulativeScoreValue_ = ((1 - alpha_) * odfSample) + (alpha_ * max);

  cumulativeScore_.addSampleToEnd(latestCumulativeScoreValue_);
}

//=======================================================================
void BTrack::predictBeat() {
  int windowSize = (int)beatPeriod_;
  double futureCumulativeScore[onsetDFBufferSize_ + windowSize];
  double w2[windowSize];

  // copy cumscore to first part of fcumscore
  for (int i = 0; i < onsetDFBufferSize_; i++) {
    futureCumulativeScore[i] = cumulativeScore_[i];
  }

  // create future window
  double v = 1;
  for (int i = 0; i < windowSize; i++) {
    w2[i] = exp((-1 * pow((v - (beatPeriod_ / 2)), 2)) /
                (2 * pow((beatPeriod_ / 2), 2)));
    v++;
  }

  // create past window
  v = -2 * beatPeriod_;
  int start = onsetDFBufferSize_ - round(2 * beatPeriod_);
  int end = onsetDFBufferSize_ - round(beatPeriod_ / 2);
  int pastwinsize = end - start + 1;
  double w1[pastwinsize];

  for (int i = 0; i < pastwinsize; i++) {
    w1[i] = exp((-1 * pow(tightness_ * log(-v / beatPeriod_), 2)) / 2);
    v = v + 1;
  }

  // calculate future cumulative score
  double max;
  int n;
  double wcumscore;
  for (int i = onsetDFBufferSize_; i < (onsetDFBufferSize_ + windowSize); i++) {
    start = i - round(2 * beatPeriod_);
    end = i - round(beatPeriod_ / 2);

    max = 0;
    n = 0;
    for (int k = start; k <= end; k++) {
      wcumscore = futureCumulativeScore[k] * w1[n];

      if (wcumscore > max) {
        max = wcumscore;
      }
      n++;
    }

    futureCumulativeScore[i] = max;
  }

  // predict beat
  max = 0;
  n = 0;

  for (int i = onsetDFBufferSize_; i < (onsetDFBufferSize_ + windowSize); i++) {
    wcumscore = futureCumulativeScore[i] * w2[n];

    if (wcumscore > max) {
      max = wcumscore;
      beatCounter_ = n;
    }

    n++;
  }

  // set next prediction time
  m0_ = beatCounter_ + round(beatPeriod_ / 2);
}