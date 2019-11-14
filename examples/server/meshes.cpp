//
// Created by Dongho Kang on 7/9/19.
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

  std::string monkeyFile = raisim::loadResource("monkey/monkey.obj");

  raisim::Mat<3, 3> inertia; inertia.setIdentity();
  const raisim::Vec<3> com = {0, 0, 0};

  int N = 3;
  double gap = 1;

  for(int row = 0; row < N; row++) {
    for(int col = 0; col < N; col++) {
      auto monkey = world.addMesh(monkeyFile, 1.0, inertia, com);
      monkey->setPosition(-gap*(N/2) + gap*row, -gap*(N/2) + gap*col, 2.0 + gap*(row*N+col));
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
