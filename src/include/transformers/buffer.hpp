#ifndef BTRACK__TRANSFORMERS__BUFFER_HPP
#define BTRACK__TRANSFORMERS__BUFFER_HPP

#include <complex>
#include <memory>
#include <vector>

namespace transformers {

class Buffer {
public:
  using Ptr = std::shared_ptr<Buffer>;
  using WeakPtr = std::weak_ptr<Buffer>;

  virtual size_t size() const = 0;
};

template <typename T> class DataBuffer : public Buffer {
public:
  using Ptr = std::shared_ptr<DataBuffer>;

  DataBuffer(std::size_t bufferSize) : data_(bufferSize, 0.0) {}

  size_t size() const override { return data_.size(); };

  std::vector<T> &data() { return data_; }
  T *raw_data() { return data_.data(); }
  // audo begin() { return data_.begin(); }
  // audo end() { return data_.end(); }

  T &operator[](std::size_t idx) { return data_[idx]; }
  const T &operator[](std::size_t idx) const { return data_[idx]; }

private:
  std::vector<T> data_;
};

using RealDataBuffer = DataBuffer<double>;
using ComplexDataBuffer = DataBuffer<std::complex<double>>;

} // namespace transformers

#endif // BTRACK__TRANSFORMERS__BUFFER_HPP