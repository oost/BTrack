#ifndef BTRACK__SRC__TRANSFORMERS__BEAT__ADAPTIVE_THRESHOLD__H_
#define BTRACK__SRC__TRANSFORMERS__BEAT__ADAPTIVE_THRESHOLD__H_

#include <memory>

#include "../../utils.h"
#include "all.h"
#include "transformers/buffers/all.h"
#include "transformers/transformers/all.h"

namespace transformers {

/*!
 * \brief Adaptive Threshold
 * Calculates an adaptive threshold which is used to remove low level energy
 * from detection function and emphasise peaks
 *
 * @param[in]   x a vector
 * @param[out]  y a vector
 */
class AdaptiveThreshold : public BufferedTransformer<double, double> {
public:
  AdaptiveThreshold(std::size_t len) : BufferedTransformer(len) {}

  virtual ~AdaptiveThreshold() {}

protected:
  void process() override {
    std::size_t i = 0;
    int k, t = 0;
    const std::vector<double> &x(input_buffer_->data());
    std::vector<double> &y(output_buffer_->data());
    std::vector<double> x_thresh(x.size());

    std::size_t p_post = 7;
    std::size_t p_pre = 8;

    t = std::min(x.size(), p_post); // what is smaller, p_post of df size. This
                                    // is to avoid accessing outside of arrays

    // find threshold for first 't' samples, where a full average cannot be
    // computed yet
    for (i = 0; i <= t; i++) {
      k = std::min((i + p_pre), x.size());
      x_thresh[i] = calculate_mean_of_array(x.begin() + 1, x.begin() + k);
    }
    // find threshold for bulk of samples across a moving average from
    // [i-p_pre,i+p_post]
    for (i = t + 1; i < x.size() - p_post; i++) {
      x_thresh[i] = calculate_mean_of_array(x.begin() + i - p_pre,
                                            x.begin() + i + p_post);
    }
    // for last few samples calculate threshold, again, not enough samples to do
    // as above
    for (i = x.size() - p_post; i < x.size(); i++) {
      k = std::max((i - p_post), std::size_t(1));
      x_thresh[i] = calculate_mean_of_array(x.begin() + k, x.end());
    }

    // subtract the threshold from the detection function and check that it is
    // not less than 0
    for (i = 0; i < x.size(); i++) {
      y[i] = y[i] - x_thresh[i];
      if (y[i] < 0) {
        y[i] = 0;
      }
    }
  }
};

} // namespace transformers

#endif // BTRACK__SRC__TRANSFORMERS__BEAT__ADAPTIVE_THRESHOLD__H_
