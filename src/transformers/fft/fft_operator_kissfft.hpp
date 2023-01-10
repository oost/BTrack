
#ifndef BTRACK__FFT_OPERATOR_KISSFFT_HPP
#define BTRACK__FFT_OPERATOR_KISSFFT_HPP

#include "kiss_fft.h"
#include "transformers/fft_operator.hpp"

namespace transformers {
class FFTOperatorKissFFT : public FFTOperator {
public:
  FFTOperatorKissFFT(int frame_size, bool backward);
  virtual ~FFTOperatorKissFFT();

private:
  void process() override;

  kiss_fft_cfg cfg_; /**< Kiss FFT configuration */
};
} // namespace transformers
#endif // BTRACK__FFT_OPERATOR_KISSFFT_HPP