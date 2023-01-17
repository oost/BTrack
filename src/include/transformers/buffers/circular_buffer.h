//=======================================================================
/** @file CircularBuffer.h
 *  @brief A class for calculating onset detection functions
 *  @author Adam Stark
 *  @copyright Copyright (C) 2008-2014  Queen Mary University of London
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//=======================================================================

#ifndef VTRACK__CIRCULAR_BUFFER_H
#define VTRACK__CIRCULAR_BUFFER_H

#include <vector>

#include "buffer.h"

//=======================================================================
/** A circular buffer that allows you to add new samples to the end
 * whilst removing them from the beginning. This is implemented in an
 * efficient way which doesn't involve any memory allocation
 */

namespace transformers {

template <typename T> class CircularBuffer : public Buffer {
public:
  using Ptr = std::shared_ptr<CircularBuffer>;

  /** Constructor */
  CircularBuffer() {}
  CircularBuffer(std::size_t len) : buffer_(len) {}

  /** Access the ith element in the buffer */
  T &operator[](int i) { return buffer_[(i + write_index_) % buffer_.size()]; }
  const T &operator[](int i) const {
    return buffer_[(i + write_index_) % buffer_.size()];
  }

  /** Add a new sample to the end of the buffer */
  void append(T v) {
    buffer_[write_index_] = v;
    write_index_ = (write_index_ + 1) % buffer_.size();
  }

  /** Resize the buffer */
  void resize(int size) {
    buffer_.resize(size);
    write_index_ = 0;
  }

  std::size_t size() const { return buffer_.size(); }

private:
  std::vector<T> buffer_;
  int write_index_ = 0;
};

} // namespace transformers

#endif /* VTRACK__CIRCULAR_BUFFER_H */
