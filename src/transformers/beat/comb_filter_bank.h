#ifndef BTRACK__SRC__TRANSFORMERS__BEAT__COMB_FILTER_BANK__H_
#define BTRACK__SRC__TRANSFORMERS__BEAT__COMB_FILTER_BANK__H_

#include <algorithm>
#include <memory>
#include <range/v3/algorithm/fill.hpp>
// #include <range/v3/range/concepts.hpp>

#include "all.h"
#include "transformers/buffers/all.h"
#include "transformers/transformers/all.h"

namespace transformers {

/*!
 * \brief Comb Filter Bank
 * Calculates the output of the comb filter bank
 *
 * @param[in]  acf  onsetDetectionFunction a pointer to an
 * array containing the onset.
 * @param[out] combBankFilterOutput  The onset detection function sample buffer
 *
 */

class CombFilterBank : public BufferedTransformer<double, double> {
public:
  static constexpr double rayparam = 43.0;

  CombFilterBank(std::size_t len)
      : BufferedTransformer(len), weighting_vector_(len),
        comb_filter_len_(len) {

    // create rayleigh weighting vector
    for (int n = 0; n < weighting_vector_.size(); n++) {
      weighting_vector_[n] =
          ((double)n / pow(rayparam, 2)) *
          exp((-1 * pow((double)-n, 2)) / (2 * pow(rayparam, 2)));
    }
  }

  virtual ~CombFilterBank() {}

protected:
  void input_updated() override {}

  void process() override {
    int numelem;

    ranges::fill(output_buffer_->data(), 0);
    // for (auto &elem : output_buffer_->data()) {
    //   elem = 0;
    // }

    numelem = 4;
    // max beat period
    for (int i = 2; i < comb_filter_len_; i++) {
      // number of comb elements
      for (int a = 1; a <= numelem; a++) {
        // general state using normalisation of comb elements
        for (int b = 1 - a; b <= a - 1; b++) {
          // calculate value for comb filter row
          (*output_buffer_)[i - 1] =
              (*output_buffer_)[i - 1] +
              ((*input_buffer_)[(a * i + b) - 1] * weighting_vector_[i - 1]) /
                  (2 * a - 1);
        }
      }
    }
  }
  int comb_filter_len_;
  std::vector<double> weighting_vector_; /**<  to hold weighting vector */
};

} // namespace transformers

#endif // BTRACK__SRC__TRANSFORMERS__BEAT__COMB_FILTER_BANK__H_