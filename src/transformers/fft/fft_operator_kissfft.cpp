#include "fft_operator_kissfft.hpp"
#include <type_traits>

using namespace transformers;

FFTOperator::Ptr FFTOperator::create_operator(int frame_size, bool backward) {
  return std::make_unique<FFTOperatorKissFFT>(frame_size, backward);
}

FFTOperatorKissFFT::FFTOperatorKissFFT(int frame_size, bool backward)
    : FFTOperator(frame_size, backward) {

  static_assert(std::is_same<double, kiss_fft_scalar>::value);

  cfg_ = kiss_fft_alloc(frame_size, backward ? 1 : 0, 0, 0);
}

//=======================================================================
FFTOperatorKissFFT::~FFTOperatorKissFFT() { free(cfg_); }

void FFTOperatorKissFFT::process() {
  kiss_fft(cfg_,
           reinterpret_cast<kiss_fft_cpx *>(input_buffer_->data()->data()),
           reinterpret_cast<kiss_fft_cpx *>(output_buffer_->data()->data()));
}