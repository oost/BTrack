#ifndef BTRACK__SRC__TRANSFORMERS__BEAT__ACF_EXTRACT_H_
#define BTRACK__SRC__TRANSFORMERS__BEAT__ACF_EXTRACT_H_

#include <algorithm>
#include <complex>
#include <memory>
#include <range/v3/algorithm/fill.hpp>
// #include <range/v3/range/concepts.hpp>

#include "all.h"
#include "transformers/buffers/all.h"
#include "transformers/transformers/all.h"

namespace transformers {

/*!
 * \brief ACFExtract
 * Puts together the results of the ACF
 *
 */

class ACFExtract : public BufferedTransformer<std::complex<double>, double> {
public:
  ACFExtract(std::size_t len) : BufferedTransformer(len), buffer_size_{len} {}

protected:
  void process() override {

    double lag = buffer_size_;
    for (int i = 0; i < buffer_size_; i++) {

      // calculate absolute value of result
      double absValue = std::abs((*input_buffer_)[i]);

      // divide by inverse lad to deal with scale bias towards small lags
      (*output_buffer_)[i] = absValue / lag;

      // this division by 1024 is technically unnecessary but it ensures the
      // algorithm produces exactly the same ACF output as the old time domain
      // implementation. The time difference is minimal so I decided to keep it
      (*output_buffer_)[i] = (*output_buffer_)[i] / 1024.;

      lag = lag - 1.;
    }
  }
  const std::size_t buffer_size_;
};

} // namespace transformers

#endif // BTRACK__SRC__TRANSFORMERS__BEAT__ACF_EXTRACT_H_