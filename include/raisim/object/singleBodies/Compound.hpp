//
// Created by jhwangbo on 18. 1. 7.
//

#ifndef RAISIM_COMPOUND_HPP
#define RAISIM_COMPOUND_HPP

#include "SingleBodyObject.hpp"
#include "raisim/math.hpp"

namespace raisim {

class Compound : public SingleBodyObject {
  friend class raisim::World;
 public:

  struct CompoundObjectChild {
    ObjectType objectType;
    Vec<4> objectParam;
    std::string material;
    Transformation trans;
  };

  Compound(const std::vector<CompoundObjectChild>& list, double mass, Mat<3,3> inertia);

  const std::vector<CompoundObjectChild>& getObjList () { return list_; };

  const std::vector<dGeomID>& getCollisionObjectList () { return co; };

 protected:

  std::vector<dGeomID> co;
  void updateCollision();
  std::vector<CompoundObjectChild> list_;
};


}
#endif //RAISIM_COMPOUND_HPP
