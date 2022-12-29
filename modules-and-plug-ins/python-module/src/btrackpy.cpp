#include "beat_tracker.hpp"
#include "onset_detection_function.hpp"

#include "btrack_constants.hpp"
#include "btrackpy.hpp"

using namespace pybind11::literals;

void init_btrackpy(py::module_ &m) {
  m.def("calculateOnsetDF", &btrack_calculateOnsetDF, "input"_a,
        "hopSize"_a = btrack_constants::default_hop_size,
        "frameSize"_a = btrack_constants::default_frame_size,
        R"pbdoc(
          Calculate the onset detection function
      )pbdoc");

  m.def("trackBeats", &btrack_trackBeats, "input"_a,
        "hopSize"_a = btrack_constants::default_hop_size,
        "frameSize"_a = btrack_constants::default_frame_size,
        R"pbdoc(
          Track beats from audio
      )pbdoc");

  m.def("trackBeatsFromOnsetDF", &btrack_trackBeatsFromOnsetDF, "input"_a,
        "hopSize"_a = btrack_constants::default_hop_size,
        "frameSize"_a = btrack_constants::default_frame_size,
        R"pbdoc(
          Track beats from an onset detection function
      )pbdoc");
}

//=======================================================================
py::array_t<double, py::array::c_style>
btrack_trackBeats(py::array_t<double> input, int hopSize, int frameSize) {
  py::buffer_info input_buffer = input.request();

  if (input_buffer.ndim != 1)
    throw std::runtime_error("Number of dimensions must be one");

  // get array size
  long signal_length = input_buffer.size;

  ////////// BEGIN PROCESS ///////////////////

  int numframes = signal_length / hopSize;
  double buffer[hopSize]; // buffer to hold one hopsize worth of audio samples

  // get number of audio frames, given the hop size and signal length

  BTrack b(hopSize, frameSize);

  int beatnum = 0;

  ///////////////////////////////////////////
  //////// Begin Processing Loop ////////////
  auto result = py::array_t<double, py::array::c_style>({numframes, 3});
  auto r = result.mutable_unchecked<2>();

  double *ptr_input = static_cast<double *>(input_buffer.ptr);

  for (int i = 0; i < numframes; i++) {
    // add new samples to frame
    for (int n = 0; n < hopSize; n++) {
      buffer[n] = ptr_input[(i * hopSize) + n];
    }

    // process the current audio frame
    b.processAudioFrame(buffer);

    // if a beat is currently scheduled
    if (b.beatDueInCurrentFrame()) {
      r(beatnum, 0) = i;
      r(beatnum, 1) = BTrack::getBeatTimeInSeconds(i, hopSize, 44100);
      r(beatnum, 2) = b.getCurrentTempoEstimate();
      beatnum = beatnum + 1;
    }
  }
  result.resize({beatnum, 3});
  return result;
}

//=======================================================================
py::array_t<double, py::array::c_style>
btrack_calculateOnsetDF(py::array_t<double> input, int hopSize, int frameSize) {
  py::buffer_info input_buffer = input.request();

  if (input_buffer.ndim != 1)
    throw std::runtime_error("Number of dimensions must be one");

  // get array size
  long signal_length = input_buffer.size;

  ////////// BEGIN PROCESS ///////////////////
  int df_type = 6;
  int numframes = signal_length / hopSize;
  double buffer[hopSize]; // buffer to hold one hopsize worth of audio samples

  OnsetDetectionFunction onset(hopSize, frameSize, df_type, 1);

  auto result = py::array_t<double>(numframes);
  py::buffer_info result_buffer = result.request();

  double *ptr_input = static_cast<double *>(input_buffer.ptr);
  double *ptr_output = static_cast<double *>(result_buffer.ptr);

  ///////////////////////////////////////////
  //////// Begin Processing Loop ////////////

  for (int i = 0; i < numframes; i++) {
    // add new samples to frame
    for (int n = 0; n < hopSize; n++) {
      buffer[n] = ptr_input[(i * hopSize) + n];
    }

    ptr_output[i] = onset.calculateOnsetDetectionFunctionSample(buffer);
  }
  return result;
}

//=======================================================================
py::array_t<double, py::array::c_style>
btrack_trackBeatsFromOnsetDF(py::array_t<double> input, int hopSize,
                             int frameSize) {
  py::buffer_info input_buffer = input.request();

  if (input_buffer.ndim != 1)
    throw std::runtime_error("Number of dimensions must be one");

  // get array size
  long numframes = input_buffer.size;

  ////////// BEGIN PROCESS ///////////////////

  BTrack b(hopSize, frameSize);

  int beatnum = 0;
  double df_val;

  ///////////////////////////////////////////
  //////// Begin Processing Loop ////////////
  auto result = py::array_t<double>(numframes);
  py::buffer_info result_buffer = result.request();

  double *ptr_input = static_cast<double *>(input_buffer.ptr);
  double *ptr_output = static_cast<double *>(result_buffer.ptr);

  for (long i = 0; i < numframes; i++) {
    df_val = ptr_input[i] + 0.0001;

    b.processOnsetDetectionFunctionSample(
        df_val); // process df sample in beat tracker

    if (b.beatDueInCurrentFrame()) {
      ptr_output[beatnum] = BTrack::getBeatTimeInSeconds(i, hopSize, 44100);
      beatnum = beatnum + 1;
    }
  }
  result.resize({beatnum});
  return result;
}
