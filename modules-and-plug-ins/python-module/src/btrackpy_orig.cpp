#include "beat_tracker.hpp"
#include "beat_tracker_original.h"
#include "onset_detection_function.h"
#include "onset_detection_function_original.h"

#include "btrackpy_orig.hpp"

using namespace pybind11::literals;
using btrack::RealArrayBuffer;
using btrack::SingleValueBuffer;

template <class B>
py::array_t<double, py::array::c_style>
btrack_orig_trackBeats(py::array_t<double> input, int hopSize, int frameSize);

template <class O>
py::array_t<double, py::array::c_style>
btrack_orig_calculateOnsetDF(py::array_t<double> input, int hopSize,
                             int frameSize);

template <class B>
py::array_t<double, py::array::c_style>
btrack_orig_trackBeatsFromOnsetDF(py::array_t<double> input, int hopSize,
                                  int frameSize);

void init_btrackpy_orig(py::module_ &m) {
  m.def("calculateOnsetDFOrig",
        &btrack_orig_calculateOnsetDF<OnsetDetectionFunctionOriginal>,
        "input"_a, "hopSize"_a = btrack_constants::default_hop_size,
        "frameSize"_a = btrack_constants::default_frame_size,
        R"pbdoc(
          Calculate the onset detection function
      )pbdoc");

  m.def("trackBeatsOrig", &btrack_orig_trackBeats<BTrackOriginal>, "input"_a,
        "hopSize"_a = btrack_constants::default_hop_size,
        "frameSize"_a = btrack_constants::default_frame_size,
        R"pbdoc(
          Track beats from audio
      )pbdoc");

  m.def("trackBeatsFromOnsetDFOrig",
        &btrack_orig_trackBeatsFromOnsetDF<BTrackOriginal>, "input"_a,
        "hopSize"_a = btrack_constants::default_hop_size,
        "frameSize"_a = btrack_constants::default_frame_size,
        R"pbdoc(
          Track beats from an onset detection function
      )pbdoc");
  m.def("calculateOnsetDFNew",
        &btrack_orig_calculateOnsetDF<btrack::OnsetDetectionFunction>,
        "input"_a, "hopSize"_a = btrack_constants::default_hop_size,
        "frameSize"_a = btrack_constants::default_frame_size,
        R"pbdoc(
          Calculate the onset detection function
      )pbdoc");

  m.def("trackBeatsNew", &btrack_orig_trackBeats<btrack::BTrack>, "input"_a,
        "hopSize"_a = btrack_constants::default_hop_size,
        "frameSize"_a = btrack_constants::default_frame_size,
        R"pbdoc(
          Track beats from audio
      )pbdoc");

  m.def("trackBeatsFromOnsetDFNew",
        &btrack_orig_trackBeatsFromOnsetDF<btrack::BTrack>, "input"_a,
        "hopSize"_a = btrack_constants::default_hop_size,
        "frameSize"_a = btrack_constants::default_frame_size,
        R"pbdoc(
          Track beats from an onset detection function
      )pbdoc");
}

//=======================================================================
template <class B>
py::array_t<double, py::array::c_style>
btrack_orig_trackBeats(py::array_t<double> input, int hopSize, int frameSize) {
  py::buffer_info input_buffer = input.request();

  if (input_buffer.ndim != 1)
    throw std::runtime_error("Number of dimensions must be one");

  // get array size
  long signal_length = input_buffer.size;

  ////////// BEGIN PROCESS ///////////////////

  int numframes = signal_length / hopSize;
  // buffer to hold one hopsize worth of audio samples
  RealArrayBuffer::Ptr btrack_input_buffer =
      std::make_shared<RealArrayBuffer>(hopSize);

  // get number of audio frames, given the hop size and signal length

  B b(hopSize, frameSize);

  int beatnum = 0;

  ///////////////////////////////////////////
  //////// Begin Processing Loop ////////////
  auto result = py::array_t<double, py::array::c_style>({numframes, 4});
  auto r = result.mutable_unchecked<2>();

  double *ptr_input = static_cast<double *>(input_buffer.ptr);

  std::vector<double> &buffer = btrack_input_buffer->data();
  for (int i = 0; i < numframes; i++) {
    // add new samples to frame
    for (int n = 0; n < hopSize; n++) {
      buffer[n] = ptr_input[(i * hopSize) + n];
    }

    // process the current audio frame
    b.process_audio_frame(buffer);

    // if a beat is currently scheduled
    if (b.beat_due_in_current_frame()) {
      r(beatnum, 0) = i;
      r(beatnum, 1) = b.get_beat_time_in_seconds(i);
      r(beatnum, 2) = b.get_current_tempo_estimate();
      r(beatnum, 3) = b.recent_average_tempo();
      beatnum = beatnum + 1;
    }
  }
  result.resize({beatnum, 4});
  return result;
}

//=======================================================================
template <class ODF>
py::array_t<double, py::array::c_style>
btrack_orig_calculateOnsetDF(py::array_t<double> input, int hopSize,
                             int frameSize) {
  py::buffer_info input_buffer = input.request();

  if (input_buffer.ndim != 1)
    throw std::runtime_error("Number of dimensions must be one");

  // get array size
  long signal_length = input_buffer.size;

  ////////// BEGIN PROCESS ///////////////////
  int df_type = 6;
  int numframes = signal_length / hopSize;

  // buffer to hold one hopsize worth of audio samples
  RealArrayBuffer::Ptr btrack_input_buffer =
      std::make_shared<RealArrayBuffer>(hopSize);

  ODF onset{hopSize, frameSize, df_type, 1};

  auto result = py::array_t<double>(numframes);
  py::buffer_info result_buffer = result.request();

  double *ptr_input = static_cast<double *>(input_buffer.ptr);
  double *ptr_output = static_cast<double *>(result_buffer.ptr);

  ///////////////////////////////////////////
  //////// Begin Processing Loop ////////////

  std::vector<double> &buffer = btrack_input_buffer->data();
  SingleValueBuffer<double>::Ptr output_ =
      onset.template output_cast<SingleValueBuffer<double>>();

  onset.set_input(btrack_input_buffer);
  for (int i = 0; i < numframes; i++) {
    // add new samples to frame
    for (int n = 0; n < hopSize; n++) {
      buffer[n] = ptr_input[(i * hopSize) + n];
    }
    onset.execute();
    ptr_output[i] = output_->value();
  }
  return result;
}

//=======================================================================
template <class B>
py::array_t<double, py::array::c_style>
btrack_orig_trackBeatsFromOnsetDF(py::array_t<double> input, int hopSize,
                                  int frameSize) {
  py::buffer_info input_buffer = input.request();

  if (input_buffer.ndim != 1)
    throw std::runtime_error("Number of dimensions must be one");

  // get array size
  long numframes = input_buffer.size;

  ////////// BEGIN PROCESS ///////////////////

  B b(hopSize, frameSize);

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

    // process df sample in beat tracker
    b.process_onset_detection_function_sample(df_val);

    if (b.beat_due_in_current_frame()) {
      ptr_output[beatnum] = b.get_beat_time_in_seconds(i);
      beatnum = beatnum + 1;
    }
  }
  result.resize({beatnum});
  return result;
}