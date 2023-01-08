#ifndef BTRACK__FFT_OPERATOR_HPP
#define BTRACK__FFT_OPERATOR_HPP

#include <complex>
#include <memory>
#include <vector>

#include "./transformers/transformer.hpp"

namespace transformers {

class FFTOperator
    : public BufferedTransformer<std::complex<double>, std::complex<double>> {
public:
  using Ptr = std::shared_ptr<FFTOperator>;
  using complex_t = std::complex<double>;
  using complex_v = std::vector<complex_t>;

  FFTOperator(int frameSize, bool backward)
      : BufferedTransformer(frameSize), backward_{backward} {};

  static Ptr createOperator(int frameSize, bool backward);

protected:
  virtual void process() = 0;

  bool backward_;
};

} // namespace transformers

#endif // BTRACK__FFT_OPERATOR_HPP
