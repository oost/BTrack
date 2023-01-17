#include "transformers/transformers/all.h"

namespace transformers {

Transformer::Ptr operator>>(Transformer::Ptr source, Transformer::Ptr sink) {
  source->add_sink(sink);
  return sink;
}

} // namespace transformers