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
class DetectionFunction : public ReductionTransformer<I, double> {
public:
  DetectionFunction() : ReductionTransformer<I, double>() {}
};

Transformer::Ptr createDetectionFunction(DetectionFunctionType function_type);

} // namespace transformers
#endif // BTRACK__DETECTION_FUNCTION_HPP