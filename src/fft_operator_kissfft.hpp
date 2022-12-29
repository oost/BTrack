
#ifndef BTRACK__FFT_OPERATOR_KISSFFT_HPP
#define BTRACK__FFT_OPERATOR_KISSFFT_HPP

#include "fft_operator.hpp"
#include "kiss_fft.h"

class FFTOperatorKissFFT : public FFTOperator {
public:
  FFTOperatorKissFFT(int frameSize, bool backward_fft);
  virtual ~FFTOperatorKissFFT();
  void performFFT(bool backward = false) override;

private:
  kiss_fft_cfg cfg_;          /**< Kiss FFT configuration */
  kiss_fft_cfg cfg_backward_; /**< Kiss FFT configuration */
};

#endif // BTRACK__FFT_OPERATOR_KISSFFT_HPP