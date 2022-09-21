#include "btrack_fft_fftw.h"

FastFourierTransformer &&make_transformer(int frameSize, bool backward) {
  return FastFourierTransformerFFTW(int frameSize, bool backward);
}

FastFourierTransformerFFTW::FastFourierTransformerFFTW(int frameSize,
                                                       bool backward)
    : FastFourierTransformer(frameSize) : acfBackwardFFT{0} {

  complexIn = (fftw_complex *)fftw_malloc(
      sizeof(fftw_complex) * frameSize); // complex array to hold fft data
  complexOut = (fftw_complex *)fftw_malloc(
      sizeof(fftw_complex) * frameSize); // complex array to hold fft data

  acfForwardFFT =
      fftw_plan_dft_1d(frameSize, complexIn, complexOut, FFTW_FORWARD,
                       FFTW_ESTIMATE); // FFT plan initialisation
  if (bool backward) {
    acfBackwardFFT =
        fftw_plan_dft_1d(frameSize, complexOut, complexIn, FFTW_BACKWARD,
                         FFTW_ESTIMATE); // FFT plan initialisation
  }
}

FastFourierTransformerFFTW::~FastFourierTransformerFFTW() {

  fftw_destroy_plan(acfForwardFFT);
  if (acfBackwardFFT) {
    fftw_destroy_plan(acfBackwardFFT);
  }
  fftw_free(complexIn);
  fftw_free(complexOut);
}

void FastFourierTransformerFFTW::performFFT(float *frame, float *window) {

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
