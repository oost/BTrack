#include <catch2/catch_all.hpp>
#include <complex>
#include <math.h>
#include <numbers>
#include <utility>

#include "transformers/fft_operator.hpp"

#ifdef USE_KISS_FFT
#include "kiss_fft.h"
#endif

#ifdef USE_FFTW
#include "fftw3.h"
#endif

using namespace std::complex_literals;
using namespace transformers;

constexpr int ULP_PRECISION = 100;

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp) {
  auto scaling = std::max(std::fabs(x + y), 1.0);
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * scaling * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
}

TEST_CASE("FFTOperator ", "[FFTOperator]") {
  int frameSize = 8;
  FFTOperator::Ptr fft_operator = FFTOperator::createOperator(frameSize, false);

  SECTION("Input and output buffers are created and have the correct size") {
    ComplexDataBuffer &output = *fft_operator->outputData();
    REQUIRE(output.size() == frameSize);
    REQUIRE(sizeof(output[0]) == 2 * sizeof(double));
  }

  SECTION("reinterpret cast works as expected") {
    ComplexDataBuffer input(2);
    input[0] = FFTOperator::complex_t(1.0, 2.0);
    input[1] = FFTOperator::complex_t(3.0, 4.0);

#ifdef USE_KISS_FFT
    kiss_fft_cpx *fftIn = reinterpret_cast<kiss_fft_cpx *>(input.rawData());
    REQUIRE(fftIn[0].r == 1.0);
    REQUIRE(fftIn[0].i == 2.0);
    REQUIRE(fftIn[1].r == 3.0);
    REQUIRE(fftIn[1].i == 4.0);
#endif

#ifdef USE_FFTW
    fftw_complex *fftIn = reinterpret_cast<fftw_complex *>(input.rawData());
    REQUIRE(fftIn[0][0] == 1.0);
    REQUIRE(fftIn[0][1] == 2.0);
    REQUIRE(fftIn[1][0] == 3.0);
    REQUIRE(fftIn[1][1] == 4.0);
#endif
  }

  SECTION("Test impulse function") {
    ComplexDataBuffer::Ptr inputBuffer =
        std::make_shared<ComplexDataBuffer>(frameSize);
    ComplexDataBuffer::Ptr output = fft_operator->outputData();

    std::fill(inputBuffer->data().begin(), inputBuffer->data().end(),
              std::complex<double>(0.0));
    (*inputBuffer)[0] = 1;

    fft_operator->setInput(inputBuffer);

    fft_operator->execute();

    // REQUIRE(output[0].real() == 0);
    std::for_each(output->data().begin(), output->data().end(),
                  [](auto v) { REQUIRE(v == 1.0); });
  }

  SECTION("Edge cases") {
    auto freq = GENERATE(range(1, 7));
    SECTION("Test exp function") {
      ComplexDataBuffer::Ptr inputBuffer =
          std::make_shared<ComplexDataBuffer>(frameSize);
      ComplexDataBuffer::Ptr output = fft_operator->outputData();

      for (int i = 0; i < frameSize; i++) {
        (*inputBuffer)[i] =
            std::exp(1i * (2 * std::numbers::pi *
                           (static_cast<double>(i) / frameSize * freq)));
      }

      fft_operator->setInput(inputBuffer);

      fft_operator->execute();

      for (std::size_t i = 0; i < output->size(); ++i) {
        if (i == freq) {
          REQUIRE(almost_equal<double>((*output)[i].real(), frameSize,
                                       ULP_PRECISION));
          REQUIRE(almost_equal((*output)[i].imag(), 0.0, ULP_PRECISION));
        } else {
          REQUIRE(almost_equal((*output)[i].real(), 0.0, ULP_PRECISION));
          REQUIRE(almost_equal((*output)[i].imag(), 0.0, ULP_PRECISION));
        }
      }
    }

    SECTION("Test shifted exp function") {
      ComplexDataBuffer::Ptr inputBuffer =
          std::make_shared<ComplexDataBuffer>(frameSize);
      ComplexDataBuffer::Ptr output = fft_operator->outputData();

      fft_operator->setInput(inputBuffer);

      for (int i = 0; i < frameSize; i++) {
        (*inputBuffer)[i] =
            std::exp(1i * (2 * std::numbers::pi *
                           (static_cast<double>(i) / frameSize * freq + 0.25)));
      }

      fft_operator->execute();

      for (std::size_t i = 0; i < output->size(); ++i) {
        if (i == freq) {
          REQUIRE(almost_equal((*output)[i].real(), 0.0, ULP_PRECISION));
          REQUIRE(almost_equal<double>((*output)[i].imag(), frameSize,
                                       ULP_PRECISION));
        } else {
          REQUIRE(almost_equal((*output)[i].real(), 0.0, ULP_PRECISION));
          REQUIRE(almost_equal((*output)[i].imag(), 0.0, ULP_PRECISION));
        }
      }
    }
  }
}