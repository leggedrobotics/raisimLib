//
// Created by jhwangbo on 30/01/18.
//

#ifndef RAISIM_COMPLIANTWIRE_HPP
#define RAISIM_COMPLIANTWIRE_HPP

#include "Wire.hpp"

namespace raisim {

class CompliantWire : public Wire {
 public:
  CompliantWire(Object* obj1, int localIdx1, Vec<3> pos1_b, Object* obj2, int localIdx2, Vec<3> pos2_b, double length, double stiffness);
  void applyTension();

 private:
  double stiffness_;
  Vec<3> tension_, tension2_;
};

}

#endif //RAISIM_COMPLIANTWIRE_HPP
