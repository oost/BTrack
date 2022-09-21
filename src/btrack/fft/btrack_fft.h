#ifndef BTRACK_BTRACK_FFT_H
#define BTRACK_BTRACK_FFT_H

#include <complex>
#include <vector>

class FastFourierTransformer {
public:
  FastFourierTransformer(int frameSize) : complex_out{frameSize} {}
  virtual ~FastFourierTransformer() = 0;
  virtual void performFFT(float *frame, float *window);

private:
  std::vector<std::complex<float>> complex_out_;
};

FastFourierTransformer *make_transformer(int frameSize, bool backward);

#endif // BTRACK_BTRACK_FFT_H