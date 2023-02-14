#ifndef BTRACK__TRANSFORMERS__BUFFERS__BUFFER_H
#define BTRACK__TRANSFORMERS__BUFFERS__BUFFER_H

#include <complex>
#include <concepts>
#include <map>
#include <memory>
#include <string_view>
#include <vector>

namespace btrack::transformers {

class Buffer {
public:
  using Ptr = std::shared_ptr<Buffer>;
  using value_t = std::nullptr_t;

  virtual ~Buffer() {}
};

template <typename T>
concept BufferObject = std::derived_from<T, Buffer>;

} // namespace btrack::transformers

#endif // BTRACK__TRANSFORMERS__BUFFERS__BUFFER_H