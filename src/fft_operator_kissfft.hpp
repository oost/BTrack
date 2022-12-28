
#ifndef BTRACK__FFT_OPERATOR_KISSFFT_HPP
#define BTRACK__FFT_OPERATOR_KISSFFT_HPP

#include "fft_operator.hpp"
#include "kiss_fft.h"

class FFTOperatorKissFFT : public FFTOperator {
public:
  FFTOperatorKissFFT(int frameSize);
  virtual ~FFTOperatorKissFFT();
  void performFFT(double *frame, double *window) override;

private:
  kiss_fft_cfg cfg;     /**< Kiss FFT configuration */
  kiss_fft_cpx *fftIn;  /**< FFT input samples, in complex form */
  kiss_fft_cpx *fftOut; /**< FFT output samples, in complex form */
  std::vector<std::vector<double>> complexOut;
};

#endif // BTRACK__FFT_OPERATOR_KISSFFT_HPP