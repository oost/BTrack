#ifndef BTRACK__SRC__TRANSFORMERS__BEAT__TEMPO_CALCULATOR__H_
#define BTRACK__SRC__TRANSFORMERS__BEAT__TEMPO_CALCULATOR__H_

#include <memory>
#include <numbers>

#include "../../utils.h"
#include "all.h"
#include "transformers/buffers/all.h"
#include "transformers/transformers/all.h"

namespace btrack::transformers {

/*!
 * \brief Tempo calculator
 * Calculates the current tempo expressed as the beat period in detection
 *
 * @param[in]  comb_filter_bank_output_
 * @param[in]  tempo_fixed_
 * @param[out] beat_period_
 * @param[out] estimated_tempo_
 *
 */

class TempoCalculator
    : public GenericTransformer<ArrayBuffer<double>, MultiBuffer> {
public:
  TempoCalculator(int sampling_rate, std::size_t hop_size, bool tempo_fixed)
      : GenericTransformer(), sampling_rate_{sampling_rate},
        hop_size_{hop_size}, tempo_fixed_{tempo_fixed} {

    tempo_to_lag_factor_ = 60.0 * static_cast<double>(sampling_rate_) / 512.;

    delta_.resize(len_);
    prev_delta_.resize(len_);
    prev_delta_fixed_.resize(len_);
    tempo_transition_matrix_.resize(len_);
    tempo_observation_vector_.resize(len_);

    // initialise prev_delta
    for (int i = 0; i < len_; i++) {
      prev_delta_[i] = 1;
    }

    initialize_transition_matrix();

    // Initialize output buffers
    beat_period_ = std::make_shared<SingleValueBuffer<double>>();
    estimated_tempo_ = std::make_shared<SingleValueBuffer<double>>();

    output_buffer_ = std::make_shared<MultiBuffer>();
    output_buffer_->add_buffer(transformers::constants::beat_period_id,
                               beat_period_);
    output_buffer_->add_buffer(transformers::constants::estimated_tempo_id,
                               estimated_tempo_);
  }

  void reset_delta(double tempo) {
    // convert tempo from bpm value to integer index of tempo probability
    int tempo_index = static_cast<int>(round((tempo - 80) / 2));

    // now set previous tempo observations to zero
    for (int i = 0; i < len_; i++) {
      prev_delta_[i] = 0;
    }

    // set desired tempo index to 1
    prev_delta_[tempo_index] = 1;
  }

private:
  void initialize_transition_matrix() {
    double t_mu = len_ / 2;
    double m_sig;
    double x;
    // create tempo transition matrix
    m_sig = len_ / 8;
    for (int i = 0; i < len_; i++) {
      tempo_transition_matrix_[i].resize(len_);
      for (int j = 0; j < len_; j++) {
        x = j + 1;
        t_mu = i + 1;
        tempo_transition_matrix_[i][j] =
            (1 / (m_sig * sqrt(2 * std::numbers::pi))) *
            exp((-1 * pow((x - t_mu), 2)) / (2 * pow(m_sig, 2)));
      }
    }
  }

  void process() override {

    int t_index;
    int t_index2;
    // calculate tempo observation vector from beat period observation vector
    for (int i = 0; i < len_; i++) {
      t_index = static_cast<int>(
          round(tempo_to_lag_factor_ / (static_cast<double>((2 * i) + 80))));
      t_index2 = static_cast<int>(
          round(tempo_to_lag_factor_ / (static_cast<double>((4 * i) + 160))));

      tempo_observation_vector_[i] =
          (*input_buffer_)[t_index - 1] + (*input_buffer_)[t_index2 - 1];
    }

    double maxval;
    double maxind;
    double curval;

    // if tempo is fixed then always use a fixed set of tempi as the previous
    // observation probability function
    if (tempo_fixed_) {
      for (int k = 0; k < len_; k++) {
        prev_delta_[k] = prev_delta_fixed_[k];
      }
    }

    for (int j = 0; j < len_; j++) {
      maxval = -1;
      for (int i = 0; i < len_; i++) {
        curval = prev_delta_[i] * tempo_transition_matrix_[i][j];

        if (curval > maxval) {
          maxval = curval;
        }
      }

      delta_[j] = maxval * tempo_observation_vector_[j];
    }

    normalize_array(delta_);

    maxind = -1;
    maxval = -1;

    for (int j = 0; j < len_; j++) {
      if (delta_[j] > maxval) {
        maxval = delta_[j];
        maxind = j;
      }

      prev_delta_[j] = delta_[j];
    }

    beat_period_->set_value(
        round((60.0 * static_cast<double>(sampling_rate_)) /
              (((2 * maxind) + 80) * static_cast<double>(hop_size_))));

    if (beat_period_ > 0) {
      estimated_tempo_->set_value(
          60.0 * static_cast<double>(sampling_rate_) /
          (static_cast<double>(hop_size_) * beat_period_->value()));
    }
  }

  const bool tempo_fixed_;
  const std::size_t hop_size_;
  const std::size_t len_ = 41;
  const int sampling_rate_;

  // Outputs
  SingleValueBuffer<double>::Ptr beat_period_;
  SingleValueBuffer<double>::Ptr estimated_tempo_;

  // Internal state
  double tempo_to_lag_factor_; /**< factor for converting between lag and
                                  tempo */
  std::vector<double> tempo_observation_vector_; /**<  to hold tempo version of
                                        comb filter output */

  std::vector<std::vector<double>>
      tempo_transition_matrix_;    /**<  tempo transition matrix */
  std::vector<double> delta_;      /**<  to hold final tempo candidate array */
  std::vector<double> prev_delta_; /**<  previous delta */
  std::vector<double>
      prev_delta_fixed_; /**<  fixed tempo version of previous delta */
};

} // namespace btrack::transformers

#endif // BTRACK__SRC__TRANSFORMERS__BEAT__TEMPO_CALCULATOR__H_