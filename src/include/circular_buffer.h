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

#include "transformers/buffer.hpp"

//=======================================================================
/** A circular buffer that allows you to add new samples to the end
 * whilst removing them from the beginning. This is implemented in an
 * efficient way which doesn't involve any memory allocation
 */

using transformers::Buffer;

template <typename T> class CircularBuffer : public Buffer {
public:
  /** Constructor */
  CircularBuffer() : write_index_(0) {}

  /** Access the ith element in the buffer */
  T &operator[](int i) {
    int index = (i + write_index_) % buffer_.size();
    return buffer_[index];
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

private:
  std::vector<T> buffer_;
  int write_index_;
};

#endif /* VTRACK__CIRCULAR_BUFFER_H */
