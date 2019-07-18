//
// Created by jhwangbo on 18. 5. 29.
//

#ifndef RAISIM_JOINTANDBODIES_HPP
#define RAISIM_JOINTANDBODIES_HPP

#include "ode/objects.h"
#include "ode/ode.h"
#include "raisim/math.hpp"
#include "raisim/object/singleBodies/Mesh.hpp"

namespace raisim {

struct CollisionDefinition {
  CollisionDefinition(const raisim::Mat<3,3>& rotOffsetI, const raisim::Vec<3>& posOffsetI,
      size_t localIdxI, dGeomID colObjI, std::string nameI) :
  rotOffset(rotOffsetI),
  posOffset(posOffsetI),
  localIdx(localIdxI),
  colObj(colObjI),
  name(std::move(nameI)) {}

  CollisionDefinition() = default;
  void setMaterial(const std::string& material);


  raisim::Mat<3,3> rotOffset;
  raisim::Vec<3> posOffset;
  size_t localIdx;
  dGeomID colObj;
  std::string name;
};

typedef std::vector<CollisionDefinition> CollisionSet;

namespace Shape {
enum Type : int {
  Box = 0,
  Cylinder,
  Sphere,
  Mesh,
  Capsule,
  Cone
};
}

struct VisObject {
  raisim::Mat<3,3> rot;
  raisim::Vec<3> offset;
  size_t localIdx;
  Shape::Type shape;
  Vec<4> color;
  std::string fileName;
  std::vector<double> visShapeParam;
  std::string material;
  std::string name;
  Vec<3> scale;
};

class Joint {
 public:

  Joint() {
    rot.setIdentity();
  }

  enum Type {
    FIXED,
    REVOLUTE,
    PRISMATIC,
    SPHERICAL,
    FLOATING
  };

  void jointAxis(std::initializer_list<double> a) {
    axis_[0] = *(a.begin());
    axis_[1] = *(a.begin() + 1);
    axis_[2] = *(a.begin() + 2);
  }

  void jointPosition(std::initializer_list<double> p) {
    pos_P_[0] = *(p.begin());
    pos_P_[1] = *(p.begin() + 1);
    pos_P_[2] = *(p.begin() + 2);
  }

  void jointPosition(const Vec<3>& p) {
    pos_P_ = p;
  }

  size_t getGcDim () {
    switch (type) {
      case FIXED:
        return 0;
      case REVOLUTE:
      case PRISMATIC:
        return 1;
      case SPHERICAL:
        return 4;
      case FLOATING:
        return 7;
    }
    return 0;
  }

  size_t getGvDim () {
    switch (type) {
      case FIXED:
        return 0;
      case REVOLUTE:
      case PRISMATIC:
        return 1;
      case SPHERICAL:
        return 3;
      case FLOATING:
        return 6;
    }
    return 0;
  }

  Vec<3> axis_;
  Vec<3> pos_P_;
  Mat<3,3> rot;
  Vec<2> limit;
  Type type;

};

class CoordinateFrame {
 public:
  Mat<3, 3> orientation;
  Vec<3> position;
  size_t parentId;
  std::string name, parentName;
  bool isChild = false; // child is the first body after movable joint. All fixed bodies attached to a child is not a child
};

class Body {

 public:
  Body() {
    mass_ = 0;
    inertia_.setZero();
    com_.setZero();
    rotMat_.setZero();
    rotMat_[0] = 1;
    rotMat_[4] = 1;
    rotMat_[8] = 1;
  }

  void mass(double mass) { mass_ = mass; }

  double &mass() { return mass_; }

  void inertia(std::initializer_list<double> inertia) {
//    LOG_IF(INFO, inertia.size() != 6) << "Provide 6 elements for inertia matrix";
    inertia_[0] = *(inertia.begin());
    inertia_[1] = *(inertia.begin() + 1);
    inertia_[2] = *(inertia.begin() + 2);

    inertia_[3] = *(inertia.begin() + 1);
    inertia_[4] = *(inertia.begin() + 3);
    inertia_[5] = *(inertia.begin() + 4);

    inertia_[6] = *(inertia.begin() + 2);
    inertia_[7] = *(inertia.begin() + 4);
    inertia_[8] = *(inertia.begin() + 5);
  }

  void inertia(Vec<3> inertia) {
//    LOG_IF(INFO, inertia.size() != 6) << "Provide 6 elements for inertia matrix";
    inertia_[0] = inertia[0];
    inertia_[1] = 0;
    inertia_[2] = 0;

    inertia_[3] = 0;
    inertia_[4] = inertia[1];
    inertia_[5] = 0;

    inertia_[6] = 0;
    inertia_[7] = 0;
    inertia_[8] = inertia[2];
  }

  void inertia(Vec<6> inertia) {
//    LOG_IF(INFO, inertia.size() != 6) << "Provide 6 elements for inertia matrix";
    inertia_[0] = inertia[0];
    inertia_[1] = inertia[1];
    inertia_[2] = inertia[2];

    inertia_[3] = inertia[1];
    inertia_[4] = inertia[3];
    inertia_[5] = inertia[4];

    inertia_[6] = inertia[2];
    inertia_[7] = inertia[4];
    inertia_[8] = inertia[5];
  }

  void setZeroInertial() {
    com_.e().setZero();
    inertia_.e().setZero();
    mass_ = 0;
  }

  Mat<3, 3> &inertia() { return inertia_; }

  void com(std::initializer_list<double> com) {
//    LOG_IF(FATAL, com.size() != 3) << "Provide 3 elements for inertia matrix";
    com_[0] = *(com.begin());
    com_[1] = *(com.begin() + 1);
    com_[2] = *(com.begin() + 2);
  }

  void com(Vec<3> com) {
//    LOG_IF(FATAL, com.size() != 3) << "Provide 3 elements for inertia matrix";
    com_ = com;
  }

  Vec<3> &com() { return com_; }

  Vec<3> &RPY() { return rollPitchYaw; }

  void RPY(const Vec<3> &rpy) {
    rollPitchYaw = rpy;
    rpyToRotMat_intrinsic(rpy, rotMat_);
  }

  Mat<3, 3> &rotationMatrix() { return rotMat_; }

  void collisionOrientation(const Vec<3> &rpy) {
    collisionRotMat.emplace_back();
    rpyToRotMat_intrinsic(rpy, collisionRotMat.back());
  }

  void collisionOrientation(const Mat<3,3> &rotMat) {
    collisionRotMat.push_back(rotMat);
  }

  void visOrientation(const Vec<3> &rpy) {
    visRotMat.emplace_back();
    rpyToRotMat_intrinsic(rpy, visRotMat.back());
  }

  void clearColAndVis() {
    colshape.clear();
    colObjOrigin.clear();
    collisionRotMat.clear();
    visshape.clear();
    visShapeParam.clear();
    visObjOrigin.clear();
    visRotMat.clear();
    meshFileNames.clear();
    visScale.clear();
    materialName.clear();
    colShapeParam.clear();
    visColor.clear();
    visName.clear();
    colName.clear();
  }

  std::vector<Shape::Type> colshape;
  std::vector<raisim::Vec<3> > colObjOrigin;
  std::vector<raisim::Mat<3, 3> > collisionRotMat;
  Vec<3> combinedColPos;
  Mat<3, 3> combinedColRotMat;
  std::vector<Shape::Type> visshape;
  std::vector<std::vector<double> > visShapeParam;
  std::vector<raisim::Vec<3> > visObjOrigin;
  std::vector<raisim::Mat<3, 3> > visRotMat;
  std::vector<raisim::Vec<4> > visColor;
  std::vector<raisim::Vec<3> > visScale;
  std::vector<std::string> meshFileNames;
  std::vector<std::vector<double> > colShapeParam;
  std::vector<std::string> materialName; /// collision property
  std::vector<std::string> collisionVisualizedMaterial; /// collision property
  std::vector<std::string> visName;
  std::vector<std::string> colName;
  std::vector<std::string> colMeshFileName;

  friend class ArticulatedSystem;
  double mass_;
  Mat<3, 3> inertia_, rotMat_, collisionRotMat_;
  Vec<3> com_, rollPitchYaw;
  bool isColliding = false;
};

class Child {
 public:

  size_t bodyIdx;
  size_t parentIdx;
  Body body;
  Joint joint;
  bool registered = false;
  std::string name, parentName;
  std::string parentJointName;

  size_t numberOfBodiesFromHere() {
    size_t nbody = 1;
    for (auto &ch : child)
      nbody += ch.numberOfBodiesFromHere();
    return nbody;
  }

  size_t numberOfDOFFromHere() {
    size_t dof = joint.getGvDim();
    for (auto &ch : child)
      dof += ch.numberOfDOFFromHere();
    return dof;
  };

  size_t numberOfGCFromHere() {
    size_t gcDim = joint.getGcDim();
    for (auto &ch : child)
      gcDim += ch.numberOfGCFromHere();
    return gcDim;
  };

  int jointIdx(std::string &nm, std::vector<std::string> &jointsNames) {
    for (uint i = 0; i < jointsNames.size(); i++)
      if (nm == jointsNames[i]) return int(i);
    return -1;
  }

  void initCollisionBodies(CollisionSet &collect, std::vector<VisObject>& visCollect, dSpaceID space, std::vector<Mesh>& mesh, const std::string& resDir);
  void initVisuals(std::vector<VisObject> &collect);

  void consumeFixedBodies(std::vector<CoordinateFrame> &frameOfInterest,
                          std::vector<std::string> &jointsNames);

  std::vector<Child> child;
  std::vector<Child> fixedBodies;
};

}

#endif //RAISIM_JOINTANDBODIES_HPP
