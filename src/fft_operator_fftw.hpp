#ifndef BTRACK__FFT_OPERATOR_FFTW_HPP
#define BTRACK__FFT_OPERATOR_FFTW_HPP

#include "fft_operator.hpp"
#include "fftw3.h"

class FFTOperatorFFTW : public FFTOperator {
public:
  FFTOperatorFFTW(int frameSize, bool backward_fft);
  virtual ~FFTOperatorFFTW();
  void performFFT(bool backward = false) override;

private:
  fftw_plan p_;          /**< fftw plan */
  fftw_plan p_backward_; /**< fftw plan */
};

#endif // BTRACK__FFT_OPERATOR_FFTW_HPP