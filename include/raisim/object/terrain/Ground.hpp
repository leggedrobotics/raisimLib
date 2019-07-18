//
// Created by kangd on 06.10.17.
//

#ifndef RAISIM_COLLISIONCHECKERBOARD_HPP
#define RAISIM_COLLISIONCHECKERBOARD_HPP

#include <memory>

#include "raisim/object/singleBodies/SingleBodyObject.hpp"

namespace raisim {

class Ground : public SingleBodyObject {

 public:
  Ground(double height);
  double getHeight() { return height_; }

 protected:
  double height_;

};

} // raisim

#endif //RAISIM_COLLISIONCHECKERBOARD_HPP
