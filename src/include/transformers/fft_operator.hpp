#ifndef BTRACK__FFT_OPERATOR_HPP
#define BTRACK__FFT_OPERATOR_HPP

#include <complex>
#include <memory>
#include <vector>

#include "./transformers/transformers/all.h"

namespace transformers {

class FFTOperator
    : public BufferedTransformer<std::complex<double>, std::complex<double>> {
public:
  using Ptr = std::shared_ptr<FFTOperator>;
  using complex_t = std::complex<double>;
  using complex_v = std::vector<complex_t>;

  FFTOperator(int frame_size, bool backward)
      : BufferedTransformer(frame_size), backward_{backward} {};

  static Ptr create_operator(int frame_size, bool backward);

protected:
  virtual void process() = 0;

  bool backward_;
};

} // namespace transformers

#endif // BTRACK__FFT_OPERATOR_HPP
