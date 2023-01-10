#ifndef BTRACK__DETECTION_FUNCTIONS_HPP
#define BTRACK__DETECTION_FUNCTIONS_HPP

#include "./complex_spectral_difference.hpp"
#include "./complex_spectral_difference_hwr.hpp"
#include "./energy_difference.hpp"
#include "./energy_envelope.hpp"
#include "./high_frequency_content.hpp"
#include "./high_frequency_spectral_difference.hpp"
#include "./high_frequency_spectral_difference_hwr.hpp"
#include "./phase_deviation.hpp"
#include "./spectral_difference.hpp"
#include "./spectral_difference_hwr.hpp"
#include "transformers/detection_function.hpp"

namespace transformers {

Transformer::Ptr create_detection_function(DetectionFunctionType function_type,
                                           std::size_t frameSize) {
  // set the window to the specified type

  switch (function_type) {
  case DetectionFunctionType::EnergyEnvelope: {
    // calculate energy envelope detection function sample
    return std::make_shared<EnergyEnvelope>();
  }
  case DetectionFunctionType::EnergyDifference: {
    // calculate half-wave rectified energy difference detection function sample
    return std::make_shared<EnergyDifference>();
  }
  case DetectionFunctionType::SpectralDifference: {
    // calculate spectral difference detection function sample
    return std::make_shared<SpectralDifference>(frameSize);
  }
  case DetectionFunctionType::SpectralDifferenceHWR: {
    // calculate spectral difference detection function sample (half wave
    // rectified)
    return std::make_shared<SpectralDifferenceHWR>(frameSize);
  }
  case DetectionFunctionType::PhaseDeviation: {
    // calculate phase deviation detection function sample (half wave rectified)
    return std::make_shared<PhaseDeviation>(frameSize);
  }
  case DetectionFunctionType::ComplexSpectralDifference: {
    // calcualte complex spectral difference detection function sample
    return std::make_shared<ComplexSpectralDifference>(frameSize);
  }
  case DetectionFunctionType::ComplexSpectralDifferenceHWR: {
    // calcualte complex spectral difference detection function sample
    // (half-wave rectified)
    return std::make_shared<ComplexSpectralDifferenceHWR>(frameSize);
  }
  case DetectionFunctionType::HighFrequencyContent: {
    // calculate high frequency content detection function sample
    return std::make_shared<HighFrequencyContent>(frameSize);
  }
  case DetectionFunctionType::HighFrequencySpectralDifference: {
    // calculate high frequency spectral difference detection function sample
    return std::make_shared<HighFrequencySpectralDifference>(frameSize);
  }
  case DetectionFunctionType::HighFrequencySpectralDifferenceHWR: {
    // calculate high frequency spectral difference detection function
    // (half-wave rectified)
    return std::make_shared<HighFrequencySpectralDifferenceHWR>(frameSize);
  }
  default: {
    throw std::runtime_error("Invalid detection function");
  }
  }
}

} // namespace transformers

#endif // BTRACK__DETECTION_FUNCTIONS_HPP