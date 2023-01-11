#ifndef BTRACK__TRANSFORMERS__BUFFER_HPP
#define BTRACK__TRANSFORMERS__BUFFER_HPP

#include <complex>
#include <map>
#include <memory>
#include <string_view>
#include <type_traits>
#include <vector>

namespace transformers {

class Buffer {
public:
  using Ptr = std::shared_ptr<Buffer>;

  virtual ~Buffer() {}
};

template <typename T>
concept BufferObject = std::is_base_of<Buffer, T>::value;

template <typename T> class DataBuffer : public Buffer {
public:
  using Ptr = std::shared_ptr<DataBuffer>;
  using value_t = T;

  DataBuffer() {}
  DataBuffer(std::size_t buffer_size) : data_(buffer_size, 0.0) {}

  void resize(size_t buffer_size) { data_.resize(buffer_size, 0.0); }
  virtual size_t size() const { return data_.size(); };

  std::vector<T> &data() { return data_; }
  T *raw_data() { return data_.data(); }

  T &operator[](std::size_t idx) { return data_[idx]; }
  const T &operator[](std::size_t idx) const { return data_[idx]; }

private:
  std::vector<T> data_;
};

using RealDataBuffer = DataBuffer<double>;
using ComplexDataBuffer = DataBuffer<std::complex<double>>;

template <typename T> class SingleValueBuffer : public Buffer {
public:
  using Ptr = std::shared_ptr<SingleValueBuffer>;
  using value_t = T;

  T &value() { return value_; }

private:
  T value_;
};

class MultiBuffer : public Buffer {
public:
  using Ptr = std::shared_ptr<MultiBuffer>;

  Buffer::Ptr buffer(const std::string &key) {
    if (auto search = buffers_.find(key); search != buffers_.end())
      return search->second;
    else
      return nullptr;
  }

  void add_buffer(const std::string &key, Buffer::Ptr buffer) {
    buffers_[key] = buffer;
  }
  size_t count() const { return buffers_.size(); }

private:
  std::map<std::string, Buffer::Ptr> buffers_;
};

} // namespace transformers

#endif // BTRACK__TRANSFORMERS__BUFFER_HPP