#include "fft_operator_fftw.hpp"

FFTOperator::Ptr FFTOperator::createOperator(int frameSize, bool backward_fft) {
  return std::make_unique<FFTOperatorFFTW>(frameSize, backward_fft);
}

FFTOperatorFFTW::FFTOperatorFFTW(int frameSize, bool backward_fft)
    : FFTOperator(frameSize, backward_fft) {

  // FFT plan initialization
  p_ = fftw_plan_dft_1d(frameSize_,
                        reinterpret_cast<fftw_complex *>(input_.data()),
                        reinterpret_cast<fftw_complex *>(output_.data()),
                        FFTW_FORWARD, FFTW_ESTIMATE);
  if (backward_fft) {
    p_backward_ = fftw_plan_dft_1d(
        frameSize_, reinterpret_cast<fftw_complex *>(output_.data()),
        reinterpret_cast<fftw_complex *>(input_.data()), FFTW_BACKWARD,
        FFTW_ESTIMATE);
  }
}

//=======================================================================
FFTOperatorFFTW::~FFTOperatorFFTW() {
  fftw_destroy_plan(p_);
  if (backward_fft_) {
    fftw_destroy_plan(p_backward_);
  }
}

void FFTOperatorFFTW::performFFT(bool backward) {
  // int fsize2 = (frameSize_ / 2);

  // window frame and copy to complex array, swapping the first and second half
  // of the signal
  // for (int i = 0; i < fsize2; i++) {
  //   complexIn_[i][0] = frame[i + fsize2] * window[i + fsize2];
  //   complexIn_[i][1] = 0.0;
  //   complexIn_[i + fsize2][0] = frame[i] * window[i];
  //   complexIn_[i + fsize2][1] = 0.0;
  // }

  // perform the fft
  if (backward) {
    fftw_execute(p_backward_);
  } else {
    fftw_execute(p_);
  }
}