//
// Created by Dongho Kang on 27/11/19.
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "helper.hpp"

int main() {

  /// create raisim world
  double dt = 0.003;

  raisim::World world;
  world.setTimeStep(dt);
  world.setERP(world.getTimeStep(), world.getTimeStep());


  /// create objects
  auto ground = world.addGround();

  std::vector<raisim::ArticulatedSystem*> anymals;

  /// ANYmal joint PD controller
  Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointForce(18), jointPgain(18), jointDgain(18);
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  jointPgain.tail(12).setConstant(200.0);
  jointDgain.tail(12).setConstant(10.0);

  const size_t N = 1;

  for(size_t i=0; i<N; i++) {
    for(size_t j=0; j<N; j++) {
      anymals.push_back(world.addArticulatedSystem(raisim::loadResource("anymal/anymal.urdf")));
      anymals.back()->setGeneralizedCoordinate({double(2*i), double(j), 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
                                                -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
      anymals.back()->setGeneralizedForce(Eigen::VectorXd::Zero(anymals.back()->getDOF()));
      anymals.back()->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
      anymals.back()->setPdGains(jointPgain, jointDgain);
      anymals.back()->setName("anymal"+std::to_string(j+i*N));
    }
  }

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.2);
  std::srand(std::time(nullptr));
  anymals.back()->printOutBodyNamesInOrder();

  /// launch raisim servear
  raisim::RaisimServer server(&world);
  server.launchServer();

  /// create visual balls
  int interval = 1200;

  for(int i = 0;; i++) {
    raisim::MSLEEP(dt*1000);

    if(i % interval == 0)
    {
      server.wakeup();
    }
    if(i % interval == interval/2)
    {
      server.hibernate();
    }

    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
