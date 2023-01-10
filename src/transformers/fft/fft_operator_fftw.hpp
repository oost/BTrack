#ifndef BTRACK__FFT_OPERATOR_FFTW_HPP
#define BTRACK__FFT_OPERATOR_FFTW_HPP

#include "fftw3.h"
#include "transformers/fft_operator.hpp"

namespace transformers {
class FFTOperatorFFTW : public FFTOperator {
public:
  FFTOperatorFFTW(int frame_size, bool backward);
  virtual ~FFTOperatorFFTW();

  void set_input(Buffer::Ptr input_buffer) override;

private:
  void prepare_plan();

  void process() override;

  fftw_plan plan_ = nullptr; /**< fftw plan */
};
} // namespace transformers

#endif // BTRACK__FFT_OPERATOR_FFTW_HPP