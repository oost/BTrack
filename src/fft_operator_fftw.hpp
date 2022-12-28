#ifndef BTRACK__FFT_OPERATOR_FFTW_HPP
#define BTRACK__FFT_OPERATOR_FFTW_HPP

#include "fft_operator.hpp"
#include "fftw3.h"

class FFTOperatorFFTW : public FFTOperator {
public:
  FFTOperatorFFTW(int frameSize);
  virtual ~FFTOperatorFFTW();
  void performFFT(const std::vector<double> &frame,
                  const std::vector<double> &window) override;

private:
  fftw_plan p;              /**< fftw plan */
  fftw_complex *complexIn;  /**< to hold complex fft values for input */
  fftw_complex *complexOut; /**< to hold complex fft values for output */
};

#endif // BTRACK__FFT_OPERATOR_FFTW_HPP