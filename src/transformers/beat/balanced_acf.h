#ifndef BTRACK__SRC__TRANSFORMERS__BEAT__BALANCED_ACF__H_
#define BTRACK__SRC__TRANSFORMERS__BEAT__BALANCED_ACF__H_

#include <memory>

#include "all.h"
#include "transformers/buffers/all.h"
#include "transformers/transformers/all.h"

namespace transformers {

/*!
 * \brief Balanced Auto Correlation
 * Calculates the balanced autocorrelation of the smoothed onset detection
 * function
 *
 * @param[in]  resampled_onset_detection_function  onsetDetectionFunction a
 * pointer to an array containing the onset.
 * @param[out] acf  The acf
 *
 */

class BalancedACF : public BufferedTransformer<double, double> {
public:
  BalancedACF(std::size_t fft_len)
      : BufferedTransformer(fft_len), fft_len_{fft_len} {
    // Set up FFT for calculating the auto-correlation function
    // FFTLengthForACFCalculation_ = 1024;

    fft_input_buffer_ =
        std::make_shared<ArrayBuffer<std::complex<double>>>(fft_len_);
    fft_operator_ = FFTOperator::create_operator(fft_len_, false);
    fft_operator_->set_input(fft_input_buffer_);

    fft_operator_backwards_ = FFTOperator::create_operator(fft_len_, true);
    fft_operator_backwards_->set_input(fft_operator_->output());
  }

  virtual ~BalancedACF() {}

protected:
  void input_updated() override {}

  void process() override {

    // copy into complex array and zero pad
    for (int i = 0; i < fft_len_; i++) {
      double v = i < input_buffer_->size() ? (*input_buffer_)[i] : 0.0;
      (*fft_input_buffer_)[i] = FFTOperator::complex_t(v, 0.0);
    }

    fft_operator_->execute();

    ComplexArrayBuffer::Ptr output = fft_operator_->output_cast();
    // multiply by complex conjugate
    for (int i = 0; i < output->size(); i++) {
      (*output)[i] = std::complex<double>(std::norm((*output)[i]), 0);
    }

    fft_operator_backwards_->execute();

    ComplexArrayBuffer::Ptr backwardOutput =
        fft_operator_backwards_->output_cast();

    double lag = 512;

    for (int i = 0; i < acf_len_; i++) {

      // calculate absolute value of result
      double absValue = std::abs((*backwardOutput)[i]);

      // divide by inverse lad to deal with scale bias towards small lags
      (*output_buffer_)[i] = absValue / lag;

      // this division by 1024 is technically unnecessary but it ensures the
      // algorithm produces exactly the same ACF output as the old time domain
      // implementation. The time difference is minimal so I decided to keep it
      (*output_buffer_)[i] = (*output_buffer_)[i] / 1024.;

      lag = lag - 1.;
    }
  }
  std::size_t acf_len_ = 512;
  std::size_t fft_len_;

  // Internal state
  FFTOperator::Ptr fft_operator_;
  FFTOperator::Ptr fft_operator_backwards_;
  ComplexArrayBuffer::Ptr fft_input_buffer_;
};

} // namespace transformers

#endif // BTRACK__SRC__TRANSFORMERS__BEAT__BALANCED_ACF__H_
