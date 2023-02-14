#ifndef BTRACK__TRANSFORMERS__BUFFERS__ARRAY_BUFFER_H
#define BTRACK__TRANSFORMERS__BUFFERS__ARRAY_BUFFER_H

#include "buffer.h"

namespace btrack::transformers {

template <typename T> class ArrayBuffer : public Buffer {
public:
  using Ptr = std::shared_ptr<ArrayBuffer>;
  using value_t = T;

  ArrayBuffer() {}
  ArrayBuffer(std::size_t buffer_size) : data_(buffer_size, 0.0) {}

  void resize(size_t buffer_size) { data_.resize(buffer_size, 0.0); }
  virtual size_t size() const { return data_.size(); };

  std::vector<T> &data() { return data_; }
  T *raw_data() { return data_.data(); }

  T &operator[](std::size_t idx) { return data_[idx]; }
  const T &operator[](std::size_t idx) const { return data_[idx]; }

private:
  std::vector<T> data_;
};

using RealArrayBuffer = ArrayBuffer<double>;
using ComplexArrayBuffer = ArrayBuffer<std::complex<double>>;

} // namespace btrack::transformers

#endif // BTRACK__TRANSFORMERS__BUFFERS__ARRAY_BUFFER_H