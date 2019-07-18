//
// Created by jhwangbo on 17. 12. 10.
//

#ifndef RAISIM_MATERIALS_HPP
#define RAISIM_MATERIALS_HPP
#include <string>
#include <vector>
#include <unordered_map>
#include "raisim_message.hpp"

namespace raisim {

struct MaterialPairProperties {
  MaterialPairProperties() = default;
  MaterialPairProperties(double c_f_, double c_r_, double r_th_) {
    c_f = c_f_;
    c_r = c_r_;
    r_th = r_th_;
  }

  double c_f = 0.8; // coefficient of friction
  double c_r = 0.0; // coefficient of restitution
  double r_th = 0.01; // restitution threshold
  double reservedForLater = 0.; // not used yet. reserved
};

class MaterialManager {

 public:

  MaterialManager() = default;

  /// upload material data from file
  explicit MaterialManager(const std::string &xmlFile);

  void setMaterialPairProp(const std::string &mat1,
                           const std::string &mat2,
                           double friction,
                           double restitution,
                           double resThreshold);

  const MaterialPairProperties &getMaterialPairProp(const std::string &mat1, const std::string &mat2);

  void setDefaultMaterialProperties (double friction, double restitution, double resThreshold);

 private:

  void init();

 public:

  std::unordered_map<unsigned int, MaterialPairProperties> materials_;
  std::unordered_map<std::string, unsigned int> materialKeys_;
  MaterialPairProperties defaultMaterial_;
  unsigned int nextMaterialIdx_ = 0;

};
}

#endif //RAISIM_MATERIALS_HPP
