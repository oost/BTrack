#ifndef SERVER__EXTERNAL__BTRACK__SRC__ORIGINAL__BTRACKORIGINAL_H_
#define SERVER__EXTERNAL__BTRACK__SRC__ORIGINAL__BTRACKORIGINAL_H_

#include "BTrack.h"
#include <span>

class BTrackOriginal : public btrack_original::BTrack {
public:
  using Ptr = std::unique_ptr<BTrackOriginal>;

  /** Constructor assuming frame size will be double the hopSize
   * @param hop_size the hop size in audio samples
   */
  BTrackOriginal(int hop_size = 512) : BTrack(hop_size) {}

  /** Constructor taking both hop_size and frame_size
   * @param hop_size the hop size in audio samples
   * @param frame_size the frame size in audio samples
   */
  BTrackOriginal(int hop_size, int frame_size) : BTrack(hop_size, frame_size) {}

  //=======================================================================
  /** Process a single audio frame
   * @param frame a pointer to an array containing an audio frame. The number of
   * samples should match the frame size that the algorithm was initialised
   * with.
   */
  void process_audio_frame(std::vector<double> &frame) {
    processAudioFrame(frame.data());
  }
  void process_audio_frame(std::span<double> frame) {
    processAudioFrame(frame.data());
  }

  //=======================================================================
  /** @returns the current hop size being used by the beat tracker */
  int get_hop_size() const { return getHopSize(); }

  /** @returns true if a beat should occur in the current audio frame */
  bool beat_due_in_current_frame() const { return beatDueInCurrentFrame(); }

  /** @returns the current tempo estimate being used by the beat tracker */
  double get_current_tempo_estimate() const {
    return getCurrentTempoEstimate();
  }

  //=======================================================================
  /** Set the tempo of the beat tracker
   * @param tempo the tempo in beats per minute (bpm)
   */
  void set_tempo(double tempo) { setTempo(tempo); }

  /** Fix tempo to roughly around some value, so that the algorithm will only
   * try to track tempi around the given tempo
   * @param tempo the tempo in beats per minute (bpm)
   */
  void fix_tempo(double tempo) { fixTempo(tempo); }

  /** Tell the algorithm to not fix the tempo anymore */
  void do_not_fix_tempo() { doNotFixTempo(); }

  //=======================================================================
  /** Calculates a beat time in seconds, given the frame number, hop size and
   * sampling frequency. This version uses a long to represent the frame number
   * @param frame_number the index of the current frame
   * @param hop_size the hop size in audio samples
   * @param fs the sampling frequency in Hz
   * @returns a beat time in seconds
   */
  double get_beat_time_in_seconds(long frame_number) const {
    return getBeatTimeInSeconds(frame_number, getHopSize(), 44100);
  }

  /** Calculates a beat time in seconds, given the frame number, hop size and
   * sampling frequency. This version uses an int to represent the frame number
   * @param frame_number the index of the current frame
   * @param hop_size the hop size in audio samples
   * @param fs the sampling frequency in Hz
   * @returns a beat time in seconds
   */
  double get_beat_time_in_seconds(int frame_number) const {
    return getBeatTimeInSeconds(frame_number, getHopSize(), 44100);
  }

  double recent_average_tempo() const { return get_current_tempo_estimate(); }

  /** Add new onset detection function sample to buffer and apply beat tracking
   * @param sample an onset detection function sample
   */
  void process_onset_detection_function_sample(double sample) {
    processOnsetDetectionFunctionSample(sample);
  }
};

#endif // SERVER__EXTERNAL__BTRACK__SRC__ORIGINAL__BTRACKORIGINAL_H_