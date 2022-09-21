#ifndef BTRACK_FFT_FFTW_H
#define BTRACK_FFT_FFTW_H

#include "btrack_fft.h"
#include "fftw3.h"

class FastFourierTransformerFFTW : FastFourierTransformer {
public:
  FastFourierTransformerFFTW(int frameSize, bool backward);
  virtual ~FastFourierTransformerFFTW();
  void performFFT(float *frame, float *window);

private:
  fftw_plan acfForwardFFT;  /**< fftw plan */
  fftw_plan acfBackwardFFT; /**< fftw plan */
  fftw_complex *complexIn;  /**< to hold complex fft values for input */
  fftw_complex *complexOut; /**< to hold complex fft values for output */
};

#endif // BTRACK_FFT_FFTW_H