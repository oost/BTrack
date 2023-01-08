#include "fft_operator_kissfft.hpp"
#include <type_traits>

using namespace transformers;

FFTOperator::Ptr FFTOperator::createOperator(int frameSize, bool backward) {
  return std::make_unique<FFTOperatorKissFFT>(frameSize, backward);
}

FFTOperatorKissFFT::FFTOperatorKissFFT(int frameSize, bool backward)
    : FFTOperator(frameSize, backward) {

  static_assert(std::is_same<double, kiss_fft_scalar>::value);

  cfg_ = kiss_fft_alloc(frameSize, backward ? 1 : 0, 0, 0);
}

//=======================================================================
FFTOperatorKissFFT::~FFTOperatorKissFFT() { free(cfg_); }

void FFTOperatorKissFFT::process(Buffer::Ptr input) {
  kiss_fft(cfg_, reinterpret_cast<kiss_fft_cpx *>(inputBuffer_->data()->data()),
           reinterpret_cast<kiss_fft_cpx *>(outputBuffer_->data()->data()));
}