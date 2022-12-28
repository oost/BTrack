#include "fft_operator_fftw.hpp"

FFTOperator::Ptr FFTOperator::createOperator(int frameSize) {
  return std::make_unique<FFTOperatorFFTW>(frameSize);
}

FFTOperatorFFTW::FFTOperatorFFTW(int frameSize) : FFTOperator(frameSize) {

  complexIn = (fftw_complex *)fftw_malloc(
      sizeof(fftw_complex) * frameSize_); // complex array to hold fft data
  output_.resize(frameSize_);
  // complexOut = (fftw_complex *)fftw_malloc(
  //     sizeof(fftw_complex) * frameSize_); // complex array to hold fft data
  p = fftw_plan_dft_1d(frameSize_, complexIn,
                       reinterpret_cast<fftw_complex *>(output_.data()),
                       FFTW_FORWARD, FFTW_ESTIMATE); // FFT
                                                     // plan
                                                     // initialisation
}

//=======================================================================
FFTOperatorFFTW::~FFTOperatorFFTW() {
  fftw_destroy_plan(p);
  fftw_free(complexIn);
}

void FFTOperatorFFTW::performFFT(const std::vector<double> &frame,
                                 const std::vector<double> &window) {
  int fsize2 = (frameSize_ / 2);

  // window frame and copy to complex array, swapping the first and second half
  // of the signal
  for (int i = 0; i < fsize2; i++) {
    complexIn[i][0] = frame[i + fsize2] * window[i + fsize2];
    complexIn[i][1] = 0.0;
    complexIn[i + fsize2][0] = frame[i] * window[i];
    complexIn[i + fsize2][1] = 0.0;
  }

  // perform the fft
  fftw_execute(p);
}