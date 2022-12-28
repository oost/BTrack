#include "beat_tracker.hpp"
#include "onset_detection_function.hpp"
// #include <numpy/arrayobject.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

constexpr int DEFAULT_HOP_SIZE = 512;
constexpr int DEFAULT_FRAME_SIZE = 1024;

namespace py = pybind11;

//=======================================================================
auto btrack_trackBeats(py::array_t<double> input, int hopSize, int frameSize) {
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
auto btrack_calculateOnsetDF(py::array_t<double> input, int hopSize,
                             int frameSize) {
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
auto btrack_trackBeatsFromOnsetDF(py::array_t<double> input, int hopSize,
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

PYBIND11_MODULE(btrackpy, m) {
  using namespace pybind11::literals;

  m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------
        .. currentmodule:: btrackpy
        .. autosummary::
           :toctree: _generate
           subtract
    )pbdoc";

  m.def("calculateOnsetDF", &btrack_calculateOnsetDF, "input"_a,
        "hopSize"_a = DEFAULT_HOP_SIZE, "frameSize"_a = DEFAULT_FRAME_SIZE,
        R"pbdoc(
          Calculate the onset detection function
      )pbdoc");

  m.def("trackBeats", &btrack_trackBeats, "input"_a,
        "hopSize"_a = DEFAULT_HOP_SIZE, "frameSize"_a = DEFAULT_FRAME_SIZE,
        R"pbdoc(
          Track beats from audio
      )pbdoc");

  m.def("trackBeatsFromOnsetDF", &btrack_trackBeatsFromOnsetDF, "input"_a,
        "hopSize"_a = DEFAULT_HOP_SIZE, "frameSize"_a = DEFAULT_FRAME_SIZE,
        R"pbdoc(
          Track beats from an onset detection function
      )pbdoc");

  m.def(
      "subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers
        Some other explanation about the subtract function.
    )pbdoc");
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
