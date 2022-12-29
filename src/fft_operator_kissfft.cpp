#include "fft_operator_kissfft.hpp"
#include <type_traits>

FFTOperator::Ptr FFTOperator::createOperator(int frameSize, bool backward_fft) {
  return std::make_unique<FFTOperatorKissFFT>(frameSize, backward_fft);
}

FFTOperatorKissFFT::FFTOperatorKissFFT(int frameSize, bool backward_fft)
    : FFTOperator(frameSize, backward_fft) {

  static_assert(std::is_same<double, kiss_fft_scalar>::value);

  cfg_ = kiss_fft_alloc(frameSize, 0, 0, 0);
  if (backward_fft) {
    cfg_backward_ = kiss_fft_alloc(frameSize, 1, 0, 0);
  }
}

//=======================================================================
FFTOperatorKissFFT::~FFTOperatorKissFFT() {
  free(cfg_);
  if (backward_fft_) {
    free(cfg_backward_);
  }
}
void FFTOperatorKissFFT::performFFT(bool backward) {
  // execute kiss fft
  if (backward) {
    kiss_fft(cfg_backward_, reinterpret_cast<kiss_fft_cpx *>(output_.data()),
             reinterpret_cast<kiss_fft_cpx *>(input_.data()));
  } else {
    kiss_fft(cfg_, reinterpret_cast<kiss_fft_cpx *>(input_.data()),
             reinterpret_cast<kiss_fft_cpx *>(output_.data()));
  }
}