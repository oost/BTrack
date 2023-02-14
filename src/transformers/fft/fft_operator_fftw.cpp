#include "fft_operator_fftw.hpp"
#include <iostream>

using namespace btrack::transformers;

FFTOperator::Ptr FFTOperator::create_operator(int frame_size, bool backward) {
  return std::make_unique<FFTOperatorFFTW>(frame_size, backward);
}

FFTOperatorFFTW::FFTOperatorFFTW(int frame_size, bool backward)
    : FFTOperator(frame_size, backward) {
  prepare_plan();
}

void FFTOperatorFFTW::set_input(Buffer::Ptr input_buffer) {
  BufferedTransformer::set_input(input_buffer);
  prepare_plan();
}

void FFTOperatorFFTW::prepare_plan() {
  if (!input_buffer_) {
    return;
  }
  if (plan_) {
    fftw_destroy_plan(plan_);
  }
  if (input_buffer_->size() != output_buffer_->size()) {
    throw std::runtime_error("Input and output sizes don't match");
  }
  if (input_buffer_->size()) {

    plan_ = fftw_plan_dft_1d(
        output_buffer_->size(),
        reinterpret_cast<fftw_complex *>(input_buffer_->raw_data()),
        reinterpret_cast<fftw_complex *>(output_buffer_->raw_data()),
        backward_ ? FFTW_BACKWARD : FFTW_FORWARD, FFTW_ESTIMATE);
  }
}

//=======================================================================
FFTOperatorFFTW::~FFTOperatorFFTW() { fftw_destroy_plan(plan_); }

void FFTOperatorFFTW::process() { fftw_execute(plan_); }
