//
// Created by jhwangbo on 18. 1. 2.
//

#ifndef RAISIM_CONFIGURE_HPP
#define RAISIM_CONFIGURE_HPP

typedef unsigned long CollisionGroup;

namespace raisim {
enum ObjectType : int { SPHERE = 0, BOX, CYLINDER, CONE, CAPSULE, MESH, HALFSPACE, COMPOUND, HEIGHTMAP, ARTICULATED_SYSTEM };

inline std::string objectTypeToString(ObjectType type) {
  switch(type) {

    case SPHERE :
      return "sphere";

    case BOX :
      return "box";

    case CYLINDER:
      return "cylinder";

    case CONE:
      return "cone";

    case CAPSULE:
      return "capsule";

    case MESH:
      return "mesh";

    case HALFSPACE:
      return "ground";

    case COMPOUND:
      return "compound";

    case HEIGHTMAP:
      return "heightmap";

    case ARTICULATED_SYSTEM:
      return "articulated_system";
  }

  return "";
}

enum class BodyType {
  STATIC = 1,
  KINEMATIC = 2,
  DYNAMIC = 3
};

}

#endif //RAISIM_CONFIGURE_HPP
