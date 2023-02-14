#ifndef BRACK__BEAT__CONSTANTS_HPP
#define BRACK__BEAT__CONSTANTS_HPP

namespace btrack::transformers::constants {

constexpr char const *beat_period_id = "beat_period";
constexpr char const *cumulative_score_id = "cumulative_score";
constexpr char const *odf_sample_id = "odf_sample";

constexpr char const *m0_id = "m0";
constexpr char const *beat_counter_id = "beat_counter";

constexpr char const *tempo_fixed_id = "tempo_fixed";
constexpr char const *comb_filter_bank_output_id = "comb_filter_bank_output";

constexpr char const *onset_df_buffer_id = "onset_df_buffer";
constexpr char const *estimated_tempo_id = "estimate_tempo";

} // namespace btrack::transformers::constants

#endif // BRACK__BEAT__CONSTANTS_HPP