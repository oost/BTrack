#ifndef BTRACK__FFT_OPERATOR_HPP
#define BTRACK__FFT_OPERATOR_HPP

#include <complex>
#include <memory>
#include <vector>

class FFTOperator {
public:
  using Ptr = std::unique_ptr<FFTOperator>;
  FFTOperator(int frameSize) : frameSize_{frameSize} {};
  virtual ~FFTOperator() {}
  virtual void performFFT(const std::vector<double> &frame,
                          const std::vector<double> &window) = 0;

  const std::vector<std::complex<double>> output() const { return output_; }

  static Ptr createOperator(int frameSize);

protected:
  int frameSize_;
  std::vector<std::complex<double>> output_;
};

#endif // BTRACK__FFT_OPERATOR_HPP
