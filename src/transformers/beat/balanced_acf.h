#ifndef BTRACK__SRC__TRANSFORMERS__BEAT__BALANCED_ACF__H_
#define BTRACK__SRC__TRANSFORMERS__BEAT__BALANCED_ACF__H_

#include <memory>

#include "../manipulation/all.h"
#include "all.h"
#include "transformers/buffers/all.h"
#include "transformers/transformer_pipeline.hpp"
#include "transformers/transformers/all.h"

namespace btrack::transformers {

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

class BalancedACF : public TransformerPipeline {
public:
  BalancedACF(std::size_t fft_len) : fft_len_{fft_len} {
    // Set up FFT for calculating the auto-correlation function
    // FFTLengthForACFCalculation_ = 1024;

    Transformer::Ptr map_operator =
        std::make_shared<MapTransformer<double, std::complex<double>>>(
            fft_len_, [](double v) { return FFTOperator::complex_t(v, 0.0); });

    Transformer::Ptr fft_operator =
        FFTOperator::create_operator(fft_len_, false);

    Transformer::Ptr norm_operator = std::make_shared<
        MapTransformer<std::complex<double>, std::complex<double>>>(
        fft_len_, [](std::complex<double> v) {
          return std::complex<double>(std::norm(v), 0);
        });

    Transformer::Ptr fft_operator_backwards =
        FFTOperator::create_operator(fft_len_, true);

    Transformer::Ptr output_map_operator =
        std::make_shared<ACFExtract>(fft_len_ / 2);

    map_operator >> fft_operator >> norm_operator >> fft_operator_backwards >>
        output_map_operator;

    set_initial_transform(map_operator);

    // Set input
  }

  virtual ~BalancedACF() {}

protected:
  std::size_t fft_len_;

  // Internal state
  // FFTOperator::Ptr fft_operator_;
  // FFTOperator::Ptr fft_operator_backwards_;
  // Transformer::Ptr map_operator_;
  // Transformer::Ptr norm_operator_;
  // Transformer::Ptr output_map_operator_;
  // ComplexArrayBuffer::Ptr fft_input_buffer_;
};

} // namespace btrack::transformers

#endif // BTRACK__SRC__TRANSFORMERS__BEAT__BALANCED_ACF__H_
