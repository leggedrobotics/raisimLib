//
// Created by jhwangbo on 01/09/17.
//

#ifndef RAISIM_ARTICULATEDSYSTEM_HPP
#define RAISIM_ARTICULATEDSYSTEM_HPP

#include <vector>
#include <initializer_list>
#include <string>
#include <array>
#include <iostream>
#include <cmath>
#include "raisim/math.hpp"
#include "algorithm"

#include "JointAndBodies.hpp"
#include "raisim/object/singleBodies/SingleBodyObject.hpp"
#include "raisim/object/singleBodies/Mesh.hpp"

namespace raisim {

class CythonArticulatedSystem;

namespace mjcf {
class LoadFromMJCF;
}

namespace urdf {
class LoadFromURDF2;
}

namespace ControlMode {
enum Type :
    int {
  FORCE_AND_TORQUE = 0,
  PD_PLUS_FEEDFORWARD_TORQUE,
  VELOCITY_PLUS_FEEDFORWARD_TORQUE
};
}

class ArticulatedSystemOption {
 public:
  bool doNotCollideWithParent = true;
};

class ArticulatedSystem :
    public Object {

  /* list of vocabs
     1. body: body here refers to only rotating bodies. Fixed bodies are optimized out. 
              Position of a body refers to the position of the joint connecting the body and its parent. 
     2. Coordinate frame: coordinate frames are defined on every joint (even at the fixed joints). If you want
                          to define a custom frame, define a fixed zero-mass object and a joint in the URDF */

 public:

  enum Frame :
      unsigned {
    WORLD_FRAME = 0,
    PARENT_FRAME,
    BODY_FRAME
  };

 private:
  friend class raisim::World;
  friend class raisim::urdf::LoadFromURDF2;
  friend class raisim::mjcf::LoadFromMJCF;
  friend class raisim::CythonArticulatedSystem;

 public:

  typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVec;
  typedef Eigen::Map<Eigen::Matrix<double, -1, -1> > EigenMat;

  ArticulatedSystem() {};

  /* Do not call this method yourself. use World class to create an Articulated system
   * @param robot description file (including the full directory)   
   * @param resource directory. if empty, using robot description folder directory
     @param joint order if you want it to be different from the order from the URDF
     @param options. see above options for details*/
  ArticulatedSystem(const std::string &filePath,
                    const std::string &resDir="",
                    std::vector<std::string> jointOrder = std::vector<std::string>(),
                    ArticulatedSystemOption options = ArticulatedSystemOption());

  ~ArticulatedSystem();

  /* Returns generalized coordinate of the system */
  const raisim::VecDyn &getGeneralizedCoordinate() { return gc_; }
  void getBaseOrientation(raisim::Vec<4>& quaternion);
  void getBaseOrientation(raisim::Mat<3,3>& rotataionMatrix) { rotataionMatrix = rot_WB[0]; }

  /* Sum of gv dim numbers of each joint (a.k.a. DoF). Joint gv dim numbers are as following
    Floating base: 6 (3 linear + 3 revolute). All velocities are defined in the world coordinate
    Revolute/Prismatic: 1
    Ball joint: 3 (a quaternion). Expressed in the world coordinate */
  const raisim::VecDyn &getGeneralizedVelocity() { return gv_; }

  /* unnecessary to call this function if you are simulating your system. integrate1 calls this function
    Call this function if you want to get kinematic properties but you don't want to integrate.  */
  void updateKinematics();

  /* gc of each joint in order. see info for getGeneralizedCoordinate() for details*/
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) { gc_ = jointState; updateKinematics(); cleanContacts(); }
  void setGeneralizedCoordinate(const raisim::VecDyn &jointState) { gc_ = jointState; updateKinematics(); cleanContacts(); }

  /* gc of each joint in order. see info for getGeneralizedVelocity() for details*/
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) { gv_ = jointVel; }
  void setGeneralizedVelocity(const raisim::VecDyn &jointVel) { gv_ = jointVel; }

  /* gc of each joint in order. see info for getGeneralizedCoordinate() for details*/
  void setGeneralizedCoordinate(std::initializer_list<double> jointState);

  /* gc of each joint in order. see info for getGeneralizedVelocity() for details*/
  void setGeneralizedVelocity(std::initializer_list<double> jointVel);

  /* This is feedforward generalized force. In the PD control mode, this differs from the actual generalizedForce
   * the dimension should be the same as dof.*/
  void setGeneralizedForce(std::initializer_list<double> tau);
  void setGeneralizedForce(const raisim::VecDyn &tau) { tauFF_ = tau; }
  void setGeneralizedForce(const Eigen::VectorXd &tau) { tauFF_ = tau; }

  /* for both the generalized coordinate and the generalized velocity */
  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) { genco = gc_.e(); genvel = gv_.e();}
  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel);

  /* get dynamics properties. Make sure that after integration you call "integrate1()" of the world object before using this method" */
  /* Generalized force is the actual force */
  const VecDyn &getGeneralizedForce() const { return tau_; }
  const VecDyn &getFeedForwardGeneralizedForce() const { return tauFF_; }
  const MatDyn &getMassMatrix() const { return M_; }
  const VecDyn &getNonlinearities() const { return h_; }
  const MatDyn &getInverseMassMatrix() const { return Minv_; }

  /* get the current center of mass of the whole system */
  const Vec<3> &getCompositeCOM() const { return composite_com_W[0]; }

  /* get the current composite inertia of the whole system.  */
  const Mat<3, 3> &getCompositeInertia() const { return compositeInertia_W[0]; }

  /* linear momentum of the whole system*/
  const Vec<3> &getLinearMomentumInCartesianSpace();

  /* returns generalized momentum. It is simply mass matrix premultiplied by the generalized velocity.
     It is already computed in "integrate1()" so you don't have to compute again. */
  const VecDyn &getGeneralizedMomentum() const { return generalizedMomentum_; }

  /* returns the sum of potential/kinetic energy given the gravitational acceleration*/
  double getEnergy(const Vec<3> &gravity);

  /* returns the kinetic energy. */
  double getKineticEnergy();

  /* returns the potential energy (relative to zero height) given the gravity vector */
  double getPotentialEnergy(const Vec<3> &gravity);

  /* bodies here means moving bodies. Fixed bodies are optimized out*/
  void printOutBodyNamesInOrder() const;

  /* frames are attached to every joint coordinate */
  void printOutFrameNamesInOrder() const;

  /* returns position in the world frame of a point defined in a joint frame*/
  void getPosition(size_t bodyIdx, const Vec<3> &point_B, Vec<3> &point_W) final;

  /* CoordinateFrame contains the pose relative to its parent expressed in the parent frame.
     If you want the position expressed in the world frame,
     you have to use this with "void getPosition_W(size_t bodyIdx, const Vec<3> &point_B, Vec<3> &point_W)".
     If you want the orientation expressed in the world frame,
     you have to get the parent body orientation and pre-multiply it by the relative orientation*/
  CoordinateFrame &getFrameByName(const std::string &nm);
  CoordinateFrame &getFrameByIdx(size_t idx);
  size_t getFrameIdxByName(const std::string &nm);
  std::vector<CoordinateFrame> &getFrames();

  /* returns position and orientation in the world frame of a frame defined in the robot description
   * Frames are attached to the joint position */
  void getFramePosition(size_t frameId, Vec<3> &point_W);
  void getFrameOrientation(size_t frameId, Mat<3, 3> &orientation_W);
  void getFrameVelocity(size_t frameId, Vec<3> &vel_W);
  void getFrameAngularVelocity(size_t frameId, Vec<3> &angVel_W);

  /* returns the position of the joint frame */
  void getPosition(size_t localIdx, Vec<3> &pos_w) final { pos_w = jointPos_W[localIdx]; }

  /* returns the orientation of the joint frame */
  void getOrientation(size_t localIdx, Mat<3, 3> &rot) final { rot = rot_WB[localIdx]; }

  /* returns the linear velocity of the joint frame*/
  void getVelocity(size_t localIdx, Vec<3> &vel_w) final { vel_w = bodyLinearVel_W[localIdx]; }

  /* Unless otherwise specified, Jacobians map the generalized velocities to the linear velocities of the given point
   expressed in the World frame. */
  void getSparseJacobian(size_t bodyIdx, const Vec<3> &point_W, SparseJacobian &jaco);

  /* TO BE TESTED*/
  void getSparseJacobianTimeDerivative(size_t bodyIdx, const Vec<3> &point_W, SparseJacobian &Dt_jaco);
  void getSparseRotationalJacobianTimeDerivative(size_t bodyIdx, const Vec<3> &point_W, SparseJacobian &Dt_jaco);

  /* returns the rotational Jacobian*/
  void getSparseRotationalJacobian(size_t bodyIdx, SparseJacobian &jaco);

  /* This method assumes that you already resized the eigen Matrix appropriately (3 by dof) and
   * the empty columns are already set to zero. */
  void convertSparseJacobianToDense(SparseJacobian &sparseJaco, Eigen::MatrixXd &denseJaco) {
    for (size_t i = 0; i < sparseJaco.size; i++) {
      denseJaco(0, sparseJaco.idx[i]) = sparseJaco[i * 3];
      denseJaco(1, sparseJaco.idx[i]) = sparseJaco[i * 3 + 1];
      denseJaco(2, sparseJaco.idx[i]) = sparseJaco[i * 3 + 2];
    }
  }

  /* This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization */
  void getDenseJacobian(size_t bodyIdx, const Vec<3> &point_W, Eigen::MatrixXd &jaco) {
    DRSFATAL_IF(jaco.rows() != 3 || jaco.cols() != dof, "Jacobian should be in size of 3XDOF")
    SparseJacobian sparseJaco;
    getSparseJacobian(bodyIdx, point_W, sparseJaco);
    for (size_t i = 0; i < jaco.size(); i++)
      for (size_t j = 0; j < 3; j++)
        jaco(j, sparseJaco.idx[i]) = sparseJaco[i * 3 + j];
  }

  /* This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization */
  void getDenseOrientationalJacobian(size_t bodyIdx, Eigen::MatrixXd &jaco) {
    DRSFATAL_IF(jaco.rows() != 3 || jaco.cols() != dof, "Jacobian should be in size of 3XDOF")
    SparseJacobian sparseJaco;
    getSparseRotationalJacobian(bodyIdx, sparseJaco);
    for (size_t i = 0; i < jaco.size(); i++)
      for (size_t j = 0; j < 3; j++)
        jaco(j, sparseJaco.idx[i]) = sparseJaco[i * 3 + j];
  }

  /* returns the velocity of the point of the jacobian*/
  void getVelocity(const SparseJacobian &jaco, Vec<3> &pointVel);

  /* returns the velocity of {a point on the body expressed} in the world frame */
  void getVelocity(size_t bodyId, const Vec<3> &posInBodyFrame, Vec<3> &pointVel);

  /* returns the angular velocity of the body*/
  void getAngularVelocity(size_t bodyId, Vec<3> &angVel);

  /* returns the index of the body */
  size_t getBodyIdx(const std::string &nm);

  /* Integrate will automatically call this method.
     This method
       1. updates kinematics if necessary (if user didn't call it manually)
       2. computes the mass matrix, nonlinear term, inverse of the mass matrix, and contact jacobians*/
  void preContactSolverUpdate1(const Vec<3> &gravity, double dt) final;

  /* returns the degrees of freedom */
  size_t getDOF() const;

  /* returns the dim of generalized coordinate */
  size_t getGeneralizedCoordinateDim() const;

  /* get both the position and orientation of a body expressed in the world frame */
  void getBodyPose(size_t bodyId, Mat<3, 3> &orientation, Vec<3> &position);

  /* The following 5 methods can be used to directly modify dynamic/kinematic properties of the robot.
     They are made for dynamic randomization. Use them with caution since they will change the
     the model permenantly. After you change the dynamic properties, call "void updateMassInfo()" to update
     some precomputed dynamic properties */
  /* returns the reference to joint position relative to its parent, expressed in the parent frame. */
  std::vector<raisim::Vec<3>> &getJointPos_P() { return jointPos_P; }

  /* returns a non-const reference to the joint mass.*/
  std::vector<double> &getMass() { return mass; }

  /* returns a non-const reference to the joint mass.*/
  std::vector<raisim::Mat<3, 3>> &getInertia() { return inertia_comB; }

  /* returns a non-const reference to the joint mass.*/
  std::vector<raisim::Vec<3>> &getLinkCOM() { return comPos_B; }

  /* returns a non-const reference to the collision bodies.*/
  raisim::CollisionSet &getCollisionBodies() { return collisionBodies; }
  raisim::CollisionDefinition &getCollisionBody(const std::string& name) {
    return *std::find_if(collisionBodies.begin(), collisionBodies.end(),
                      [name](const raisim::CollisionDefinition& ref){ return ref.name == name; }); }

  /* You must call this function after you change dynamic parameters. */
  void updateMassInfo();

  /* get link mass */
  double getMass(size_t localIdx) final { return mass[localIdx]; }

  /* set link mass */
  void setMass(size_t localIdx, double value) { mass[localIdx] = value; }

  /* get the total mass*/
  double getTotalMass() { return compositeMass[0]; }

  /* set external forces or torques expressed in the world frame acting on the COM of the body */
  void setExternalForce(size_t localIdx, const Vec<3> &force) final;

  void setExternalTorque(size_t localIdx, const Vec<3> &torque) final;

  /* set external force acting on the point specified*/
  void setExternalForce(size_t localIdx, Frame frameOfForce, const Vec<3> &force, Frame frameOfPos, const Vec<3> &pos);

  /* returns the contact point velocity */
  void getContactPointVel(size_t pointId, Vec<3> &vel) final;

  /* control methods. They can be specified in the description file as well */
  void setControlMode(ControlMode::Type mode) { controlMode_ = mode; }

  ControlMode::Type getControlMode() { return controlMode_; }

  /* set PD targets. It is effective only in the control mode "PD_PLUS_FEEDFORWARD_TORQUE".
     set any arbitrary number for unactuated degrees of freedom */
  void setPdTarget(const Eigen::VectorXd &posTarget, const Eigen::VectorXd &velTarget) {
    qref_ = posTarget;
    uref_ = velTarget;
  }

  void setPdTarget(const raisim::VecDyn &posTarget, const raisim::VecDyn &velTarget) {
    RSFATAL_IF(posTarget.n != gcDim,"position target should have the same dimension as the generalized coordinate")
    RSFATAL_IF(velTarget.n != dof, "the velocity target should have the same dimension as the degrees of freedom")
    qref_ = posTarget;
    uref_ = velTarget;
  }

  /* set PD gains. It is effective only in the control mode "PD_PLUS_FEEDFORWARD_TORQUE".
     set zero to unactuated degrees of freemdom. Both have dimensions equal to DOF*/
  void setPdGains(const Eigen::VectorXd &pgain, const Eigen::VectorXd &dgain) {
    RSFATAL_IF(pgain.rows()!=dof || dgain.rows() != dof, "p/d gains should have the same dimension as the degrees of freedom")
    kp_ = pgain;
    kd_ = dgain;
    dampedDiagonalTermUpdated_ = false;
  }

  void setPdGains(const raisim::VecDyn &pgain, const raisim::VecDyn &dgain) {
    RSFATAL_IF(pgain.n !=dof || dgain.n != dof, "p/d gains should have the same dimension as the degrees of freedom")
    kp_ = pgain;
    kd_ = dgain;
    dampedDiagonalTermUpdated_ = false;
  }

  /* passive elements at the joints. They can be specified in the URDF file as well */
  void setJointDamping(const Eigen::VectorXd &dampingCoefficient) {
    cd_ = dampingCoefficient;
    dampedDiagonalTermUpdated_ = false;
  }

  void setJointDamping(const raisim::VecDyn &dampingCoefficient) {
    cd_ = dampingCoefficient;
    dampedDiagonalTermUpdated_ = false;
  }

  /* This computes the inverse mass matrix given the mass matrix. The return type is dense.
     It exploits the sparsity of the mass matrix to efficiently perform the computation. */
  void computeSparseInverse(const MatDyn &M, MatDyn &Minv);

  /* this method exploits the sparsity of the mass matrix. If the mass matrix is
     nearly dense, it will be slower than your ordinary matrix multiplication
     which is probably vectorized */
  inline void massMatrixVecMul(const VecDyn &vec1, VecDyn &vec);

  void ignoreCollisionBetween(size_t bodyIdx1, size_t bodyIdx2);

  /* return options. currently only supports "DO_NOT_COLLIDE_WITH_PARENT"*/
  ArticulatedSystemOption getOptions() { return options_; }

  const std::vector<std::string> &getBodyNames() { return bodyName; }

  std::vector<VisObject> &getVisOb() { return visObj; };

  std::vector<VisObject> &getVisColOb() { return visColObj; };

  void getVisObPose(size_t bodyIdx, Mat<3,3>& rot, Vec<3>& pos) { getPose(visObj[bodyIdx], rot, pos); }

  void getVisColObPose(size_t bodyIdx, Mat<3,3>& rot, Vec<3>& pos) { getPose(visColObj[bodyIdx], rot, pos); }

  const std::string &getResourceDir() { return resourceDir_; }

  // name of the robot definition file name
  const std::string &getRobotDescriptionfFileName() { return robotDefFileName_; }

  // name of the directory containing the robot definition file
  const std::string &getRobotDescriptionfTopDirName() { return robotDefFileUpperDir_; }

  /* change the base position and orientation of the base. */
  /* eigen methods. _e is for removing abiguity */
  void setBasePos_e(const Eigen::Vector3d& pos);
  void setBaseOrientation_e(const Eigen::Matrix3d& rot);
  /* raisim vec methods */
  void setBasePos(const Vec<3>& pos);
  void setBaseOrientation(const Mat<3,3>& rot);

  /* set limit in actuation force. */
  void setActuationLimits(const Eigen::VectorXd& upper, const Eigen::VectorXd& lower) {
    tauUpper_ = upper; tauLower_ = lower;
  }

  /* change collision geom parameters. Recreate the visual object since it only updates the simulation object*/
  void setCollisionObjectShapeParameters(size_t id, const std::vector<double>& params);
  void setCollisionObjectPositionOffset(size_t id, const Vec<3>& posOffset);
  void setCollisionObjectOrientationOffset(size_t id, const Mat<3,3>& oriOffset);

  /* rotor inertia is a term added to the diagonal of the mass matrix. This approximates the rotor inertia. Note that this
   * is not exactly equivalent in dynamics (due to gyroscopic effect). but it is a commonly used approximation. */
  void setRotorInertia(const VecDyn& rotorInertia) {
    rotorInertia_ = rotorInertia;
  }

 protected:

  void getPose(const VisObject& vob, Mat<3,3>& rot, Vec<3>& pos);

  void getSparseJacobian_internal(size_t bodyIdx, const Vec<3> &point_W, SparseJacobian &jaco);

  CollisionSet &getCollisionObj() { return collisionBodies; }

  void updateCollision() final;

  ObjectType getObjectType() final { return ARTICULATED_SYSTEM; }

  BodyType getBodyType(size_t localIdx) const final;

  BodyType getBodyType() const final { return BodyType::DYNAMIC; };

  void computeMassMatrix(MatDyn &M);

  void computeNonlinearities(const Vec<3> &gravity, VecDyn &b);

  void destroyCollisionBodies(dSpaceID id) final;

  /* computing JM^-1J^T exploiting sparsity */
  void getFullDelassusAndTauStar(double dt);

  /* This computes Delassus matrix necessary for contact force computation */
  void preContactSolverUpdate2(const Vec<3> &gravity, double dt) final;

  void integrate(double dt) final;

  void addContactPointVel(size_t pointId, Vec<3> &vel) final;

  void subContactPointVel(size_t pointId, Vec<3> &vel) final;

  void updateGenVelWithImpulse(size_t pointId, const Vec<3> &imp) final;

  void updateTimeStep(double dt) final;

  void updateTimeStepIfNecessary(double dt) final;

 private:

  void init();

  void integratePosition(double dt);

  void computeExpandedParentArray();

  void appendJointLimits(contact::ContactProblems &problem) final;

  void enforceJointLimits(contact::Single3DContactProblem &problem) final;

  void computeDampedMass(double dt);

  void cleanContacts() { contactJaco_.clear(); getContacts().clear(); }

  /// for computation
  /// Frames:: W: world, B: body, P: parent

  /// changing variables
  std::vector<raisim::Mat<3, 3>> rot_WB;
  std::vector<raisim::Mat<3, 3>> rot_JC;
  std::vector<raisim::Vec<3>> jointAxis_W;
  std::vector<raisim::Vec<3>> jointPos_W;
  std::vector<raisim::Vec<3>> joint2com_W;
  std::vector<raisim::Vec<3>> comPos_W;
  std::vector<raisim::Mat<3, 3>> compositeInertia_W;
  std::vector<raisim::Vec<3>> composite_mXCOM_W;
  std::vector<raisim::Vec<3>> composite_com_W;
  std::vector<raisim::Vec<3>> bodyLinearVel_W;
  std::vector<raisim::Vec<3>> bodyAngVel_W, bodyAngVel_B;
  std::vector<raisim::Vec<3>> bodyAngVel_pc_W;
  std::vector<raisim::Vec<3>> bodyLinVel_pc_W;
  std::vector<raisim::Vec<3>> bodyLinearAcc;
  std::vector<raisim::Vec<3>> bodyAngAcc;
  std::vector<raisim::Vec<3>> propagatingForceAtJoint_W;
  std::vector<raisim::Vec<3>> propagatingTorqueAtJoint_W;
  std::vector<raisim::Mat<3, 3>> inertiaAboutJoint_W;
  std::vector<raisim::Mat<3, 3>> inertia_comW;
  std::vector<raisim::Vec<3>> sketch;
  std::vector<raisim::Vec<3>> joint2joint_W;

  /// constant params
  std::vector<raisim::Mat<3, 3>> rot_JB, rot_WJ;
  std::vector<raisim::Vec<3>> jointPos_P;
  std::vector<raisim::Vec<3>> jointAxis_P;
  std::vector<raisim::Mat<3, 3>> inertia_comB;
  std::vector<raisim::Vec<3>> comPos_B;
  std::vector<raisim::Vec<2>> jointLimits_;
  Vec<3> fixedBasePos_;

  std::vector<Joint::Type> jointType;
  std::vector<size_t> jointGcDim_;
  std::vector<size_t> jointGvDim_;

  std::vector<double> mass;
  std::vector<size_t> parent;
  std::vector<size_t> leafnodes_;
  std::vector<double> compositeMass;

  std::vector<std::vector<size_t> > children_;
  std::vector<size_t> toBaseBodyCount_; // counts to base (which is WORLD).
  std::vector<size_t> toBaseGvDimCount_;
  std::vector<size_t> toBaseGcDimCount_;
  std::vector<size_t> lambda;

  VecDyn gc_, gcOld_, gv_, gvOld_, gvAvg_, h_, gvERP_, gvTemp_;
  MatDyn M_, Minv_, lT_;

  std::vector<MatDyn> MinvJT_T;
  VecDyn tauStar_, tau_, tauFF_; // tau_ is the actual torque applied to the joints. tauFF_ is the feedforward joint torque. these two can be differ if the control mode is not FORCE_AND_TORQUE
  VecDyn tauUpper_, tauLower_; // bounds
  std::vector<size_t> bodyIdx2GvIdx, bodyIdx2GcIdx;

  std::vector<SparseJacobian> J_;

  std::vector<CoordinateFrame> frameOfInterest_;
  std::vector<std::string> jointsNames_;

  /// total linear momentum in cartesian space
  Vec<3> linearMomentum_;

 protected:
  std::vector<Child> child_;
  std::vector<std::string> bodyName;
  std::vector<SparseJacobian> contactJaco_;
  std::vector<SparseJacobian> externalForceJaco_;
  std::vector<raisim::Vec<3>> externalForces_;

  raisim::CollisionSet collisionBodies;
  std::vector<VisObject> visColObj, visObj;
  ArticulatedSystemOption options_;

 private:
  size_t nbody, dof = 0, gcDim = 0, baseDOFminusOne = 0;
  bool hasSphericalJoint_ = false;
  bool kinematicsUpdated_ = false;
  bool dampedDiagonalTermUpdated_ = false;
  ControlMode::Type controlMode_ = ControlMode::FORCE_AND_TORQUE;

  // damping
  VecDyn cd_;

  struct SpringElement {
    size_t childBodyId = 0;
    size_t stiffness = 0;
    double mountAngle = 0;
  };

  std::vector<SpringElement> springs_;

  // stable PD controller
  VecDyn kp_, kd_, uref_, qref_, diag_w, posErr_;
  VecDyn rotorInertia_;
  VecDyn generalizedMomentum_;
  VecDyn temp, temp2;
  MatDyn temp1;
  std::string resourceDir_, robotDefFileName_, robotDefFileUpperDir_;

  // for Trimesh
  std::vector<dTriMeshDataID> meshData_;
  std::vector<std::vector<float>> meshVertices_;
  std::vector<std::vector<dTriIndex>> meshIdx_;

};

}

#endif //RAISIM_ARTICULATEDSYSTEM_HPP
