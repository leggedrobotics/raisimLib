//
// Created by kangd on 05.10.17.
//

#ifndef RAISIM_WORLD_HPP
#define RAISIM_WORLD_HPP

#include <memory>
#include <algorithm>

#include "raisim/helper.hpp"
#include "raisim/object/Object.hpp"
#include "raisim/object/singleBodies/SingleBodyObject.hpp"
#include <raisim/object/singleBodies/Sphere.hpp>
#include <raisim/object/singleBodies/Cylinder.hpp>
#include <raisim/object/singleBodies/Box.hpp>
#include <raisim/object/singleBodies/Capsule.hpp>
#include <raisim/object/singleBodies/Cone.hpp>
#include <raisim/object/singleBodies/Compound.hpp>
#include <raisim/constraints/StiffWire.hpp>
#include <raisim/constraints/CompliantWire.hpp>
#include "raisim/object/terrain/Ground.hpp"
#include "raisim/object/terrain/HeightMap.hpp"
#include "raisim/Terrain.hpp"
#include "raisim/contact/BisectionContactSolver.hpp"
#include "raisim/object/ArticulatedSystem/ArticulatedSystem.hpp"
#include "ode/collision.h"
#include "configure.hpp"

namespace raisim {

class World {

 public:

  typedef std::vector<contact::Single3DContactProblem, AlignedAllocator<contact::Single3DContactProblem, 32>>
      ContactProblems;

  explicit World();
  explicit World(const std::string &configFile);

  World(const World &world) = delete;

  ~World();

  void setTimeStep(double dt) {
    timeStep_ = dt;
    solver_.setTimestep(dt);
    for (auto &ob: objectList_) ob->updateTimeStep(dt);
  }

  double getTimeStep() const { return timeStep_; }

  /** to dynamically add objects */
  Sphere *addSphere(double radius,
                    double mass,
                    const std::string &material = "default",
                    CollisionGroup collisionGroup = 1,
                    CollisionGroup collisionMask = CollisionGroup(-1));

  Box *addBox(double xLength,
              double yLength,
              double zLength,
              double mass,
              const std::string &material = "default",
              CollisionGroup collisionGroup = 1,
              CollisionGroup collisionMask = CollisionGroup(-1));

  Cylinder *addCylinder(double radius,
                        double height,
                        double mass,
                        const std::string &material = "default",
                        CollisionGroup collisionGroup = 1,
                        CollisionGroup collisionMask = CollisionGroup(-1));

  Cone *addCone(double radius,
                double height,
                double mass,
                const std::string &material = "default",
                CollisionGroup collisionGroup = 1,
                CollisionGroup collisionMask = CollisionGroup(-1));

  Capsule *addCapsule(double radius,
                      double height,
                      double mass,
                      const std::string &material = "default",
                      CollisionGroup collisionGroup = 1,
                      CollisionGroup collisionMask = CollisionGroup(-1));

  Ground *addGround(double zHeight = 0.0,
                    const std::string &material = "default",
                    CollisionGroup collisionMask = CollisionGroup(-1));

  HeightMap *addHeightMap(int xSamples,
                          int ysamples,
                          double xScale,
                          double yScale,
                          double centerX,
                          double centerY,
                          const std::vector<double> &height,
                          const std::string &material = "default",
                          CollisionGroup collisionGroup = 1,
                          CollisionGroup collisionMask = CollisionGroup(-1));

  HeightMap *addHeightMap(const std::string &raisimHeightMapFileName,
                          double centerX,
                          double centerY,
                          const std::string &material = "default",
                          CollisionGroup collisionGroup = 1,
                          CollisionGroup collisionMask = CollisionGroup(-1));

  HeightMap *addHeightMap(const std::string &pngFileName,
                          double centerX,
                          double centerY,
                          double xSize,
                          double ySize,
                          double heightScale,
                          double heightOffset,
                          const std::string &material = "default",
                          CollisionGroup collisionGroup = 1,
                          CollisionGroup collisionMask = CollisionGroup(-1));

  HeightMap *addHeightMap(double centerX,
                          double centerY,
                          TerrainProperties &terrainProperties,
                          const std::string &material = "default",
                          CollisionGroup collisionGroup = 1,
                          CollisionGroup collisionMask = CollisionGroup(-1));

  /* @param path to urdf file or a URDF string
   * @param path to the resource directory. leave it empty ("") if it is the urdf file directory
   * @param collisionGroup
   * @param collisionMask
   * @param option*/
  ArticulatedSystem *addArticulatedSystem(const std::string &filePathOrURDFScript,
                                          const std::string &resPath = "",
                                          const std::vector<std::string> &jointOrder = {},
                                          CollisionGroup collisionGroup = 1,
                                          CollisionGroup collisionMask = CollisionGroup(-1),
                                          ArticulatedSystemOption options = ArticulatedSystemOption());

  Compound *addCompound(const std::vector<Compound::CompoundObjectChild> &children,
                        double mass,
                        const Mat<3, 3>& inertia,
                        CollisionGroup collisionGroup = 1,
                        CollisionGroup collisionMask = CollisionGroup(-1));

  StiffWire *addStiffWire(Object *obj1,
                          size_t localIdx1,
                          Vec<3> pos1_b,
                          Object *obj2,
                          size_t localIdx2,
                          Vec<3> pos2_b,
                          double length);

  CompliantWire *addCompliantWire(Object *obj1,
                                  int localIdx1,
                                  Vec<3> pos1_b,
                                  Object *obj2,
                                  int localIdx2,
                                  Vec<3> pos2_b,
                                  double length,
                                  double stiffness);

  Mesh* addMesh(const std::string& fileName,
                double mass,
                const Mat<3, 3>& inertia,
                const Vec<3>& COM,
                const std::string& material = "",
                CollisionGroup collisionGroup = 1,
                CollisionGroup collisionMask = CollisionGroup(-1));

  /* returns nullptr if it doesn't find the object */
  Object *getObject(const std::string &name);
  raisim::Object* getObject(std::size_t worldIndex) { return objectList_[worldIndex]; }

  /* returns nullptr if it doesn't find the constraint */
  Constraints *getConstraint(const std::string &name);

  /* returns nullptr if it doesn't find the constraint */
  Wire *getWire(const std::string &name);

  /* this number is updated everytime that you add or remove object. */
  unsigned long getConfigurationNumber() { return objectConfiguration_; }

  /** to dynamically remove objects */
  void removeObject(Object *obj);
  void removeObject(StiffWire *wire);
  void removeObject(CompliantWire *wire);

  /** this function is simply calling both "integrate1()" and "integrate2()" one-by-one*/
  void integrate();

  /** It performs
        1) deletion contacts from previous time step
        2) collision detection 
        3) register contacts to each body
        4) calls "preContactSolverUpdate1()" of each object */
  void integrate1();

  /** It performs
        1) calls "preContactSolverUpdate2()" of each body
        2) run collision solver
        3) calls "integrate" method of each object */
  void integrate2();

  const ContactProblems *getContactProblem() const { return &contactProblems_; }
  std::vector<Object *> &getObjList();

  void setGravity(const Vec<3> &gravity);
  void updateMaterialProp(const MaterialManager &prop);

  void setMaterialPairProp(const std::string &mat1,
                           const std::string &mat2,
                           double friction,
                           double restitution,
                           double resThreshold);

  void setDefaultMaterial(double friction,
                          double restitution,
                          double resThreshold);

  const Vec<3> &getGravity() { return gravity_; }
  void setERP(double erp, double erp2 = 0);
  void setContactSolverParam(double alpha_init, double alpha_min, double alpha_decay, int maxIter, double threshold);

  /** return the total integrated time (which is updated at every integrate2() call)*/
  double getWorldTime() const { return worldTime_; }
  void setWorldTime(double time) { worldTime_ = time; }

  raisim::contact::BisectionContactSolver &getContactSolver() { return solver_; }
  const raisim::contact::BisectionContactSolver &getContactSolver() const { return solver_; }

 protected:
  void contactProblemUpdate(Object *objectA);
  void addCollisionObject(dGeomID colObj,
                          size_t localIdx,
                          const std::string &material,
                          CollisionGroup collisionGroup,
                          CollisionGroup collisionMask);
  dSpaceID collisionWorld_;
  std::pair<std::vector<dContactGeom>, int> contacts_;
  std::vector<dGeomID> colObjList_;

  // simulation properties
  Vec<3> gravity_;

  // contact solver
  raisim::contact::BisectionContactSolver solver_;

  // list
  std::vector<Object *> objectList_;
  ContactProblems contactProblems_;
  std::vector<int> colIdxToObjIdx_;
  std::vector<int> colIdxToLocalObjIdx_;

  // constraints
  std::vector<std::unique_ptr<StiffWire>> stiffWire_;
  std::vector<std::unique_ptr<CompliantWire>> compliantWire_;

  MaterialManager mat_;
  MaterialPairProperties defaultMaterialProperty_;
  double timeStep_ = 0.005;
  double worldTime_ = 0.;

  // bookkeeping
  unsigned long stepsTaken_ = 0, objectConfiguration_ = 0;
  void updateObjConfiig() { objectConfiguration_++; }
  void updateStepCount() { stepsTaken_++; }
};

} // raisim

#endif //RAISIM_WORLD_HPP
