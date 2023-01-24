#include "transformers/transformer_pipeline.hpp"
#include "transformers/transformers/all.h"
namespace transformers {

Transformer::Ptr operator>>(Transformer::Ptr source, Transformer::Ptr sink) {
  source->add_sink(sink);
  return sink;
}

Transformer::Ptr operator>>(Buffer::Ptr input_buffer,
                            Transformer::Ptr transformer) {
  transformer->set_input(input_buffer);
  return transformer;
}

Transformer::Ptr operator>>(TransformerPipeline::Ptr pipeline,
                            Transformer::Ptr initial_transform) {
  pipeline->set_initial_transform(initial_transform);
  return initial_transform;
}

} // namespace transformers