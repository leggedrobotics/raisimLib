//
// Created by Dongho Kang on 4/9/19.
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
//  world.setERP(world.getTimeStep(), world.getTimeStep());


  /// create objects
  auto ground = world.addGround();

  std::vector<raisim::Box*> cubes;
  std::vector<raisim::Sphere*> spheres;
  std::vector<raisim::Capsule*> capsules;
  std::vector<raisim::Cylinder*> cylinders;


  static const int N=3;

  for(size_t i=0; i<N; i++) {
    for(size_t j=0; j<N; j++) {
      for (size_t k = 0; k < N; k++) {
        std::string number = std::to_string(i) + std::to_string(j) + std::to_string(k);
        raisim::SingleBodyObject *ob = nullptr;
        switch ((i + j + k) % 4) {
          case 0:
            cubes.push_back(world.addBox(1, 1, 1, 1));
            ob = cubes.back();
            break;
          case 1:
            spheres.push_back(world.addSphere(0.5, 1));
            ob = spheres.back();
            break;
          case 2:
            capsules.push_back(world.addCapsule(0.5, 1., 1));
            ob = capsules.back();
            break;
          case 3:
            cylinders.push_back(world.addCylinder(0.5, 0.5, 1));
            ob = cylinders.back();
            break;
        }
        ob->setPosition(-N + 2.*i, -N + 2.*j, N*2. + 2.*k);
      }
    }
  }


  /// launch raisim servear
  raisim::RaisimServer server(&world);
  server.launchServer();

  while(1) {
    raisim::MSLEEP(2);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
