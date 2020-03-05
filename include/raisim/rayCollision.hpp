//
// Created by jemin on 2/20/20.
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef RAISIM_RAYCOLLISION_HPP
#define RAISIM_RAYCOLLISION_HPP

#include "raisim/object/Object.hpp"
#include "ode/collision.h"

namespace raisim {

class RayCollisionItem {

 public:
  friend class raisim::World;

  Object* getObject() {
    return obj;
  }

  const Eigen::Vector3d& getPosition() {
    return pos;
  }

 protected:
  dContactGeom geoms;

 private:

  Eigen::Vector3d pos;
  Object* obj;
};

class RayCollisionList {

 public:

  class iterator
  {
   public:
    typedef iterator self_type;
    typedef RayCollisionItem value_type;
    typedef RayCollisionItem& reference;
    typedef RayCollisionItem* pointer;
    typedef std::forward_iterator_tag iterator_category;
    typedef int difference_type;
    explicit iterator(pointer ptr) : ptr_(ptr) { }
    self_type operator++() { self_type i = *this; ptr_++; return i; }
    self_type operator++(int junk) { ptr_++; return *this; }
    reference operator*() { return *ptr_; }
    pointer operator->() { return ptr_; }
    bool operator==(const self_type& rhs) { return ptr_ == rhs.ptr_; }
    bool operator!=(const self_type& rhs) { return ptr_ != rhs.ptr_; }
   private:
    pointer ptr_;
  };

  friend class raisim::World;

  RayCollisionItem & operator [](size_t i) {return list_[i];}
  const RayCollisionItem & operator [](size_t i) const {return list_[i];}

  size_t size() {
    return size_;
  }

  iterator begin() {
    return iterator(&list_[0]);
  }

  iterator end() {
    return iterator(&list_[size_-1]);
  }

  void setSize(size_t size) {
    size_ = size;
  }

  void resize(size_t size) {
    list_.resize(size);
  }

  void reserve(size_t size) {
    list_.reserve(size);
  }

 protected:
  std::vector<RayCollisionItem> list_;
  size_t size_;
};
}

#endif //RAISIM_RAYCOLLISION_HPP
