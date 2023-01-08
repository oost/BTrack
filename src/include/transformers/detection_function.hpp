#ifndef BTRACK__DETECTION_FUNCTION_HPP
#define BTRACK__DETECTION_FUNCTION_HPP

#include <complex>
#include <memory>

#include "transformers/transformer.hpp"

namespace transformers {

//=======================================================================
/** The type of onset detection function to calculate */
enum class DetectionFunctionType {
  EnergyEnvelope,
  EnergyDifference,
  SpectralDifference,
  SpectralDifferenceHWR,
  PhaseDeviation,
  ComplexSpectralDifference,
  ComplexSpectralDifferenceHWR,
  HighFrequencyContent,
  HighFrequencySpectralDifference,
  HighFrequencySpectralDifferenceHWR
};

template <typename I>
class DetectionFunction : public BufferedTransformer<I, double> {
public:
  DetectionFunction() : BufferedTransformer<I, double>(1) {}
};

Transformer::Ptr createDetectionFunction(DetectionFunctionType function_type);

} // namespace transformers
#endif // BTRACK__DETECTION_FUNCTION_HPP