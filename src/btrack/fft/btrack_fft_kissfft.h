#ifndef BTRACK_FFT_KISSFFT_H
#define BTRACK_FFT_KISSFFT_H

#include <vector>

#include "btrack_fft.h"
#include "kiss_fft.h"

class FastFourierTransformerKISSFFT : FastFourierTransformer {
public:
  FastFourierTransformerKISSFFT(int frameSize, bool backward);
  virtual ~FastFourierTransformerKISSFFT();
  void performFFT(float *frame, float *window);

private:
  kiss_fft_cfg cfg;     /**< Kiss FFT configuration */
  kiss_fft_cpx *fftIn;  /**< FFT input samples, in complex form */
  kiss_fft_cpx *fftOut; /**< FFT output samples, in complex form */
};

#endif // BTRACK_FFT_KISSFFT_H