//
// Created by Jhwangbo on 05.10.17.
//

#ifndef RAISIM_RAICOLLISIONMESH_HPP
#define RAISIM_RAICOLLISIONMESH_HPP

#include "SingleBodyObject.hpp"

namespace raisim {

class Mesh : public SingleBodyObject {

 public:
  explicit Mesh(const std::string& filename, dSpaceID space, double mass, const Mat<3,3>& inertia, const Vec<3>& COM);

  explicit Mesh(const std::string& filename, dSpaceID space);

 protected:

  void loadObj(const std::string& filename);

  std::vector<uint32_t> idx_;
  std::vector<double> verticies_;

};

} // raisim

#endif //RAISIM_RAICOLLISIONSPHERE_HPP
