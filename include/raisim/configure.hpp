//
// Created by jhwangbo on 18. 1. 2.
//

#ifndef RAISIM_CONFIGURE_HPP
#define RAISIM_CONFIGURE_HPP

typedef unsigned long CollisionGroup;

namespace raisim {
enum ObjectType : int { SPHERE = 0, BOX, CYLINDER, CONE, CAPSULE, MESH, HALFSPACE, COMPOUND, HEIGHTMAP, ARTICULATED_SYSTEM };

enum class BodyType {
  STATIC = 1,
  KINEMATIC = 2,
  DYNAMIC = 3
};

}

#endif //RAISIM_CONFIGURE_HPP
