//
// Created by kangd on 06.10.17.
//

#ifndef RAISIM_COLLISIONCYLINDER_HPP
#define RAISIM_COLLISIONCYLINDER_HPP

#include "SingleBodyObject.hpp"

namespace raisim {

class Cylinder : public SingleBodyObject {

  /// NOTE
  /// body frame origin of Cylinder is C.O.M of Cylinder
 public:
  Cylinder(double radius, double height, double mass);

  float getRadius() const;
  float getHeight() const;

 protected:

  float radius_;
  float height_;
};

} // raisim

#endif //RAISIM_COLLISIONCYLINDER_HPP
