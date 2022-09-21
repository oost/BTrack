#include "btrack_fft_kissfft.h"

FastFourierTransformer &&make_transformer(int frameSize, bool backward) {
  return FastFourierTransformerKISSFFT(int frameSize, bool backward);
}

FastFourierTransformerKISSFFT::FastFourierTransformerKISSFFT(int frameSize,
                                                             bool backward)
    : cfgBackwards{0} {
  fftIn = new kiss_fft_cpx[frameSize];
  fftOut = new kiss_fft_cpx[frameSize];
  cfgForwards = kiss_fft_alloc(frameSize, 0, 0, 0);
  if (backward) {
    cfgBackwards = kiss_fft_alloc(frameSize, 1, 0, 0);
  }
}

FastFourierTransformerKISSFFT::~FastFourierTransformerKISSFFT() {
  free(cfgForwards);
  if (cfgBackwards) {
    free(cfgBackwards);
  }
  delete[] fftIn;
  delete[] fftOut;
}

void FastFourierTransformerKISSFFT::performFFT(float *frame, float *window) {
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
    complex_out_[i].real(fftOut[i].r);
    complex_out_[i].imag(fftOut[i].i);
  }
}
