//
// Created by kangd on 19.11.17.
//

#ifndef RAISIM_CONTACT_HPP
#define RAISIM_CONTACT_HPP

#include <raisim/configure.hpp>

namespace raisim {
namespace contact {

class Contact {
 public:
  explicit Contact(Vec<3> &position,
                   Vec<3> &normal,
                   bool objectA,
                   size_t contactProblemIndex,
                   size_t contactIndexInObject,
                   size_t pairObjectIndex,
                   BodyType pairObjectBodyType,
                   size_t pairContactIndexInPairObject,
                   size_t localBodyIndex,
                   double depth)
    : position_(position), normal_(normal), objectA_(objectA),
      pairObjectBodyType_(pairObjectBodyType),
      contactIndexInObject_(contactIndexInObject),
      contactProblemIndex_(contactProblemIndex),
      pairObjectIndex_(pairObjectIndex),
      pairContactIndexInPairObject_(pairContactIndexInPairObject),
      localBodyIndex_(localBodyIndex),
      depth_(depth) {

    // set contact frame
    // contact frame is R_PiW not R_WPi
    raisim::Vec<3> &zAxis = normal_;
    raisim::Vec<3> xAxis;
    raisim::Vec<3> yAxis;

    if (zAxis[0] == 0.0 && zAxis[1] == 0.0)
      xAxis = {1.0, 0.0, 0.0};
    else
      xAxis = {zAxis[1], -zAxis[0], 0};
    cross(zAxis, xAxis, yAxis);

    const double xNorm = xAxis.norm();
    const double yNorm = yAxis.norm();
    const double zNorm = zAxis.norm();

    xAxis = {xAxis[0] / xNorm, xAxis[1] / xNorm, xAxis[2] / xNorm};
    yAxis = {yAxis[0] / yNorm, yAxis[1] / yNorm, yAxis[2] / yNorm};
    zAxis = {zAxis[0] / zNorm, zAxis[1] / zNorm, zAxis[2] / zNorm};

    frame_[0] = xAxis[0];
    frame_[1] = yAxis[0];
    frame_[2] = zAxis[0];
    frame_[3] = xAxis[1];
    frame_[4] = yAxis[1];
    frame_[5] = zAxis[1];
    frame_[6] = xAxis[2];
    frame_[7] = yAxis[2];
    frame_[8] = zAxis[2];
  }

  const Vec<3> &getPosition() const {
    return position_;
  }

  const Vec<3> &getNormal() const {
    return normal_;
  }

  const Mat<3, 3> &getContactFrame() const {
    return frame_;
  }

  size_t getIndexContactProblem() const {
    return contactProblemIndex_;
  }

  size_t getPairObjectIndex() const {
    return pairObjectIndex_;
  }

  size_t getPairContactIndexInPairObject() const {
    return pairContactIndexInPairObject_;
  }

  Vec<3> *getImpulse() const {
    return impulse_;
  }

  bool isObjectA() const {
    return objectA_;
  }

  BodyType getPairObjectBodyType() const {
    return pairObjectBodyType_;
  }

  void setImpulse(Vec<3> *impulse) {
    impulse_ = impulse;
  }

  void setInvInertia(Mat<3, 3> *appIinv) {
    appInertiaInv_ = appIinv;
  }

  const Mat<3, 3> *getInvInertia() const {
    return appInertiaInv_;
  }

  size_t getlocalBodyIndex() const {
    return localBodyIndex_;
  }

  double getDepth() const {
    return depth_;
  }

  const bool isSelfCollision() const {
    return isSelfCollision_;
  }

  void setSelfCollision() {
    isSelfCollision_ = true;
  }

  const bool skip() const {
    return skip_;
  }

  void setSkip() {
    skip_ = true;
  }

 private:
  Mat<3, 3> frame_;              // contactFrame of A
  Vec<3> position_;             // position of A = position of B
  Vec<3> normal_;               // normal of A (normal of B = - normalA)
  Vec<3> *impulse_;
  Mat<3, 3> *appInertiaInv_;
  double depth_;
  size_t contactIndexInObject_;
  size_t pairObjectIndex_;
  size_t pairContactIndexInPairObject_;
  size_t contactProblemIndex_;
  size_t localBodyIndex_ = 0;
  bool isSelfCollision_ = false;
  bool skip_ = false;
  bool objectA_;                                  // true (A) / false (B)
  BodyType pairObjectBodyType_;

};

} // contact
} // raisim

#endif //RAISIM_CONTACT_HPP
