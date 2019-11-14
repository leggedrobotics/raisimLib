//
// Created by Dongho Kang on 6/9/19.
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
  raisim::World world;
  world.setTimeStep(0.003);
  world.setERP(world.getTimeStep(), world.getTimeStep());


  /// create objects
  auto ground = world.addGround();

  std::vector<raisim::ArticulatedSystem*> atlas;

  const size_t N = 1;

  for(size_t i=0; i<N; i++) {
    for(size_t j=0; j<N; j++) {
      atlas.push_back(world.addArticulatedSystem(raisim::loadResource("atlas/robot.urdf")));
      atlas.back()->setGeneralizedCoordinate({double(2 * i), double(j), 1.0, 1.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      atlas.back()->setGeneralizedForce(Eigen::VectorXd::Zero(atlas.back()->getDOF()));
      atlas.back()->setName("atlas" + std::to_string(j + i * N));
    }
  }

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.2);
  std::srand(std::time(nullptr));
  atlas.back()->printOutBodyNamesInOrder();

  /// launch raisim servear
  raisim::RaisimServer server(&world);
  server.launchServer();

  while(1) {
    raisim::MSLEEP(2);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
