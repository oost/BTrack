#ifndef BTRACK__TRANSFORMERS__BUFFERS__MULTI_BUFFER_H
#define BTRACK__TRANSFORMERS__BUFFERS__MULTI_BUFFER_H

#include "buffer.h"

namespace btrack::transformers {

class MultiBuffer : public Buffer {
public:
  using Ptr = std::shared_ptr<MultiBuffer>;

  Buffer::Ptr buffer(const std::string &key) {
    if (auto search = buffers_.find(key); search != buffers_.end())
      return search->second;
    else
      return nullptr;
  }

  template <class B> std::shared_ptr<B> buffer_cast(const std::string &key) {
    return std::dynamic_pointer_cast<B>(this->buffer(key));
  }

  template <class T>
  void add_buffer(const std::string &key, T buffer)
    requires std::derived_from<typename T::element_type, Buffer>
  {
    buffers_[key] = buffer;
  }

  template <class T, class... Args>
  void make_buffer(const std::string &key, Args &&...args)
    requires std::derived_from<typename T::element_type, Buffer>
  {
    buffers_[key] = std::make_shared<T>(std::forward<Args>(args)...);
  }

  size_t count() const { return buffers_.size(); }

private:
  std::map<std::string, Buffer::Ptr> buffers_;
};

} // namespace btrack::transformers

#endif // BTRACK__TRANSFORMERS__BUFFERS__MULTI_BUFFER_H