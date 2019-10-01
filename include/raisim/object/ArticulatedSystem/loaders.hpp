//
// Created by jhwangbo on 18. 5. 29.
//

#ifndef RAISIM_LOADERS_HPP
#define RAISIM_LOADERS_HPP

#include "ArticulatedSystem.hpp"
#include <map>

namespace raisim {

inline int countNumOfSpaces(const char *str) {
  int iSpaces = 0;
  for (unsigned int iLoop = 0; iLoop < (sizeof(str) / sizeof(str[0])); iLoop++)
    if (str[iLoop] == ' ')
      iSpaces++;
  return iSpaces;
}

inline double firstNumber(const char *txt) {
  double num;
  std::string s(txt);
  std::string delimiter = " ";

  while (s.substr(0, 1) == " ")
    s = s.substr(1, s.size());
  std::string token = s.substr(0, s.find(delimiter));
  num = std::stod(token);

  return num;
}

inline Vec<3> char2Vec3(const char *txt) {
  Vec<3> vec;
  std::string s(txt);
  std::string delimiter = " ";

  for (int i = 0; i < 3; i++) {
    while (s.substr(0, 1) == " ")
      s = s.substr(1, s.size());
    std::string token = s.substr(0, s.find(delimiter));
    vec[i] = std::stod(token);
    if (i != 2) s.erase(0, s.find(delimiter) + delimiter.length());
  }
  return vec;
}

inline Vec<2> char2Vec2(const char *txt) {
  Vec<2> vec;
  std::string s(txt);
  std::string delimiter = " ";

  for (int i = 0; i < 2; i++) {
    while (s.substr(0, 1) == " ")
      s = s.substr(1, s.size());
    std::string token = s.substr(0, s.find(delimiter));
    vec[i] = std::stod(token);
    if (i != 1) s.erase(0, s.find(delimiter) + delimiter.length());
  }
  return vec;
}

inline Vec<6> char2Vec6(const char *txt) {
  Vec<6> vec;
  std::string s(txt);
  std::string delimiter = " ";

  for (int i = 0; i < 6; i++) {
    while (s.substr(0, 1) == " ")
      s = s.substr(1, s.size());

    std::string token = s.substr(0, s.find(delimiter));
    vec[i] = std::stod(token);
    if (i != 5) s.erase(0, s.find(delimiter) + delimiter.length());
  }
  return vec;
}

inline Vec<4> char2Vec4(const char *txt) {
  Vec<4> vec;
  std::string s(txt);
  std::string delimiter = " ";

  for (int i = 0; i < 4; i++) {
    while (s.substr(0, 1) == " ")
      s = s.substr(1, s.size());
    std::string token = s.substr(0, s.find(delimiter));
    vec[i] = std::stod(token);
    if (i != 3) s.erase(0, s.find(delimiter) + delimiter.length());
  }
  return vec;
}

namespace object {
class ArticulatedSystem;
}

namespace urdf {

struct UrdfMaterial {
  UrdfMaterial() {
    color = {0.7, 0.7, 0.7};
  }
  std::string name;
  Vec<3> color;
};

inline Shape::Type charToGeom(const char *txt) {
  std::string s(txt);
  if (s == "box")
    return Shape::Box;
  else if (s == "mesh")
    return Shape::Mesh;
  else if (s == "cylinder")
    return Shape::Cylinder;
  else if (s == "capsule")
    return Shape::Capsule;
  else if (s == "sphere")
    return Shape::Sphere;
  else RSFATAL("unsupported shape: "<<s <<". all characters should be lowercase")

  return Shape::Box;
}

struct UrdfBody {
  UrdfBody() {
    origin.setZero();
    rot.setIdentity();
    scale = {1.,1.,1.};
  };

  Shape::Type shape;
  std::string fileName;
  Vec<3> origin;
  Mat<3, 3> rot;
  Vec<3> scale;
  std::vector<double> param;
  std::string mat;
  std::string collision_mat;
  std::string name;
};

struct UrdfLinkInertial {
  UrdfLinkInertial() {
    inertia.setZero();
    origin.setZero();
    rot.setIdentity();
  }

  Vec<3> origin;
  Mat<3, 3> rot;
  double mass = 0;
  Mat<3, 3> inertia;
};

struct UrdfJoint {
  UrdfJoint() {
    limit.setZero();
    origin.setZero();
    axis[0] = 1; axis[1] = 0; axis[2] = 0;
  }
  std::string name = "", parent, child;
  Joint::Type type;
  Vec<3> origin;
  Mat<3, 3> rot;
  Vec<3> axis;
  Vec<2> limit;
  double damping = 0;
  double stiffness = 0, springMountPos = 0;
};

struct UrdfLink {
  std::string name;
  UrdfJoint parentJoint;
  UrdfLink* parent = nullptr;
  std::vector<UrdfLink *> child;
  std::vector<UrdfBody> visual, collision;
  UrdfLinkInertial inertial;
  Vec<4> color_;
};

class LoadFromURDF2 {

 public:
  LoadFromURDF2(ArticulatedSystem &system, std::string filePath, std::vector<std::string> jointOrder, bool isItAFilePath);
 private:
  void processLinkFromUrdf(const UrdfLink *urdfLink,
                           Child &raiLink,
                           const std::vector<std::string> &jointsOrder);
  std::map<std::string, UrdfMaterial> mats;
  std::string currentObject_;


};
}

namespace mjcf {
class LoadFromMJCF {
 public:
  LoadFromMJCF(ArticulatedSystem &system, std::string filePath, std::vector<std::string> jointOrder);
};
}
}

#endif //RAISIM_LOADERS_HPP
