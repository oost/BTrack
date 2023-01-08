#include "fft_operator_fftw.hpp"
#include <iostream>

using namespace transformers;

FFTOperator::Ptr FFTOperator::createOperator(int frameSize, bool backward) {
  return std::make_unique<FFTOperatorFFTW>(frameSize, backward);
}

FFTOperatorFFTW::FFTOperatorFFTW(int frameSize, bool backward)
    : FFTOperator(frameSize, backward) {
  prepare_plan();
}

void FFTOperatorFFTW::setInput(Buffer::Ptr inputBuffer) {
  BufferedTransformer::setInput(inputBuffer);
  prepare_plan();
}

void FFTOperatorFFTW::prepare_plan() {
  if (!inputBuffer_) {
    return;
  }
  if (plan_) {
    fftw_destroy_plan(plan_);
  }
  if (inputBuffer_->size() != outputBuffer_->size()) {
    throw std::runtime_error("Input and output sizes don't match");
  }
  if (inputBuffer_->size()) {

    plan_ = fftw_plan_dft_1d(
        outputBuffer_->size(),
        reinterpret_cast<fftw_complex *>(inputBuffer_->rawData()),
        reinterpret_cast<fftw_complex *>(outputBuffer_->rawData()),
        backward_ ? FFTW_BACKWARD : FFTW_FORWARD, FFTW_ESTIMATE);
  }
}

//=======================================================================
FFTOperatorFFTW::~FFTOperatorFFTW() { fftw_destroy_plan(plan_); }

void FFTOperatorFFTW::process() { fftw_execute(plan_); }
