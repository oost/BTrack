#ifndef BTRACK__FFT_OPERATOR_HPP
#define BTRACK__FFT_OPERATOR_HPP

#include <complex>
#include <memory>
#include <vector>

class FFTOperator {
public:
  using Ptr = std::unique_ptr<FFTOperator>;
  using complex_t = std::complex<double>;
  using complex_v = std::vector<complex_t>;

  FFTOperator(int frameSize, bool backward_fft)
      : frameSize_{frameSize}, backward_fft_{backward_fft} {
    output_.resize(frameSize);
    input_.resize(frameSize);
  };
  virtual ~FFTOperator() {}
  virtual void performFFT(bool backward = false) = 0;

  complex_v &output() { return output_; }
  complex_v &input() { return input_; }

  static Ptr createOperator(int frameSize, bool backward_fft);

protected:
  int frameSize_;
  bool backward_fft_;
  complex_v output_;
  complex_v input_;
};

#endif // BTRACK__FFT_OPERATOR_HPP
