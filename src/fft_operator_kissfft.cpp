#include "fft_operator_kissfft.hpp"

FFTOperator::Ptr FFTOperator::createOperator(int frameSize) {
  return std::make_unique<FFTOperatorKissFFT>(frameSize);
}

FFTOperatorKissFFT::FFTOperatorKissFFT(int frameSize) : FFTOperator(frameSize) {

  complexOut.resize(frameSize_);

  for (int i = 0; i < frameSize_; i++) {
    complexOut[i].resize(2);
  }

  fftIn = new kiss_fft_cpx[frameSize_];
  fftOut = new kiss_fft_cpx[frameSize_];
  cfg = kiss_fft_alloc(frameSize, 0, 0, 0);
}

//=======================================================================
FFTOperatorKissFFT::~FFTOperatorKissFFT() {
  free(cfg);
  delete[] fftIn;
  delete[] fftOut;
}

void FFTOperatorFFTW::performFFT(double *frame, double *window) {
  int fsize2 = (frameSize_ / 2);

  for (int i = 0; i < fsize2; i++) {
    fftIn[i].r = frame[i + fsize2] * window[i + fsize2];
    fftIn[i].i = 0.0;
    fftIn[i + fsize2].r = frame[i] * window[i];
    fftIn[i + fsize2].i = 0.0;
  }

  // execute kiss fft
  kiss_fft(cfg, fftIn, fftOut);

  // store real and imaginary parts of FFT
  for (int i = 0; i < frameSize; i++) {
    complexOut[i][0] = fftOut[i].r;
    complexOut[i][1] = fftOut[i].i;
  }
}