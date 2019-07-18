//
// Created by jemin on 3/2/19.
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

#ifndef RAISIM_RAISIMSERVER_HPP
#define RAISIM_RAISIMSERVER_HPP

#ifdef __linux__
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#elif WINDOWS

#endif

#include "World.hpp"
#include "helper.hpp"
#include "object/ArticulatedSystem/JointAndBodies.hpp"
#include <mutex>
#include <thread>
#include <string>
#include <fstream>
#include <atomic>
#include <sys/wait.h>

namespace raisim {

class RaisimServer {

 public:

  static constexpr int SEND_BUFFER_SIZE = 33554432;
  static constexpr int MAXIMUM_PACKET_SIZE = 4096;
  static constexpr int FOOTER_SIZE = sizeof(char);
  static constexpr int RECEIVE_BUFFER_SIZE = 1024;
  static constexpr int RAISIM_PORT = 8080;

  enum ClientMessageType : int {
    REQUEST_OBJECT_POSITION = 0,
    REQUEST_INITIALIZATION,
    REQUEST_RESOURCE, // request mesh, texture. etc files
    REQUEST_CHANGE_REALTIME_FACTOR,
    REQUEST_CONTACT_SOLVER_DETAILS,
    REQUEST_PAUSE,
    REQUEST_RESUME
  };

  enum ServerMessageType : int {
    INITIALIZATION = 0,
    OBJECT_POSITION_UPDATE = 1,
    STATUS = 2,
    NO_MESSAGE
  };

  enum Status : int {
    STATUS_RENDERING = 0,
    STATUS_HIBERNATING = 1,
    STATUS_TERMINATING = 2
  };

  explicit RaisimServer(World *world) : world_(world) {
    receive_buffer.resize(RECEIVE_BUFFER_SIZE);
    send_buffer.resize(SEND_BUFFER_SIZE);
  }

  void loop() {
    int opt = 1;
    int addrlen = sizeof(address);

    // Creating socket file descriptor
    RSFATAL_IF((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0, "socket failed")
    RSFATAL_IF(setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)), "setsockopt")

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(RAISIM_PORT);

    // Forcefully attaching socket to the port 8080
    RSFATAL_IF(bind(server_fd, (struct sockaddr *) &address, sizeof(address)) < 0, "bind failed")
    RSFATAL_IF(listen(server_fd, 3) < 0, "listen failed")

    while(!terminateRequested_) {
      if(waitForReadEvent(0.01)) {
        RSFATAL_IF((client_ = accept(server_fd, (struct sockaddr *) &address, (socklen_t *) &addrlen)) < 0, "accept failed")
        connected_ = true;
      }

      while (connected_) {
        if(terminateRequested_) {
          state_ = STATUS_TERMINATING;
          connected_ = false;
        }
        connected_ = processRequests();
      }
    }
    close(server_fd);
    state_ = STATUS_RENDERING;
  }

  template<typename T>
  static inline char *set(char *data, T *val) {
    memcpy(data, val, sizeof(T));
    data += sizeof(T);
    return data;
  }

  template<typename T>
  static inline char *setN(char *data, T *val, size_t N) {
    memcpy(data, val, sizeof(T) * N);
    data += sizeof(T) * N;
    return data;
  }

  template<typename T>
  static inline char *set(char *data, T val) {
    T temp = val;
    memcpy(data, &temp, sizeof(T));
    data += sizeof(T);
    return data;
  }

  template<typename T>
  static inline char *get(char *data, T *val) {
    memcpy(val, data, sizeof(T));
    data += sizeof(T);
    return data;
  }

  template<typename T>
  static inline char *getN(char *data, T *val, size_t N) {
    memcpy(val, data, sizeof(T)*N);
    data += sizeof(T)*N;
    return data;
  }

  static inline char *getString(char *data, std::string& str) {
    size_t size;
    data = get(data, &size);
    str.resize(size);
    data = getN(data, const_cast<char*>(str.c_str()), size);
    return data;
  }

  static inline char *setString(char *data, const std::string& val) {
    data = set(data, val.size());
    memcpy(data, val.data(), sizeof(char)*val.size());
    data += sizeof(char) * val.size();
    return data;
  }

  template<typename T>
  static inline char *setStdVector(char *data, const std::vector<T>& str) {
    data = set(data, str.size());
    data = setN(data, str.data(), str.size());
    return data;
  }

  template<typename T>
  static inline char *getStdVector(char *data, std::vector<T>& str) {
    size_t size;
    data = get(data, &size);
    str.resize(size);
    data = getN(data, str.data(), str.size());
    return data;
  }

  void launchServer() {
    serverThread_ = std::thread(&raisim::RaisimServer::loop, this);
  }

  void integrateWorldThreadSafe() {    
    std::lock_guard<std::mutex> guard(worldMutex_);
    world_->integrate();
    updateReady_ = true;
  }

  void killServer() {
    terminateRequested_ = true;
    serverThread_.join();
    terminateRequested_ = false;
  }

  void informClientForUpdate() { updateReady_ = true; }

  void lockWorldMutex() { worldMutex_.lock(); }

  void unlockWorldMutex() { worldMutex_.unlock(); }

  bool isPauseRequested() { return pauseRequested_; }

  bool isResumeRuested() { return resumeRequested_; }

  bool isTerminateRequested() { return terminateRequested_; }

 private:

  bool processRequests() {
    data_ = &send_buffer[0];
    ClientMessageType type;

    if(recv(client_, &receive_buffer[0], MAXIMUM_PACKET_SIZE, 0) == 0) {
      return false;
    }

    get(&receive_buffer[0], &type);

    data_ = set(data_, state_);

    switch (type) {
      case REQUEST_OBJECT_POSITION:
        serializeWorld();
        break;

      case REQUEST_INITIALIZATION:
        serializeObjects();
        break;

      case REQUEST_CHANGE_REALTIME_FACTOR:
        changeRealTimeFactor();
        break;
    }

    bool eom = false;
    char* startPtr = &send_buffer[0];

    while(!eom) {
      if (data_ - startPtr > MAXIMUM_PACKET_SIZE - FOOTER_SIZE) {
        memcpy(&tempBuffer[0], startPtr, MAXIMUM_PACKET_SIZE - FOOTER_SIZE);
        tempBuffer[MAXIMUM_PACKET_SIZE-FOOTER_SIZE] = 'c';
        send(client_, &tempBuffer[0], MAXIMUM_PACKET_SIZE, 0);
        startPtr += MAXIMUM_PACKET_SIZE - FOOTER_SIZE;
      } else {
        memcpy(&tempBuffer[0], startPtr, data_ - startPtr);
        tempBuffer[MAXIMUM_PACKET_SIZE-FOOTER_SIZE] = 'e';
        send(client_, &tempBuffer[0], data_ - startPtr, 0);
        eom = true;
      }
    }
    return state_ == STATUS_RENDERING;
  }

  void serializeWorld() {

    if(!updateReady_) {
      data_ = set(data_, ServerMessageType::NO_MESSAGE);
      return;
    }

    std::lock_guard<std::mutex> guard(worldMutex_);

    auto &objList = world_->getObjList();

    // set message type
    data_ = set(data_, ServerMessageType::OBJECT_POSITION_UPDATE);

    data_ = set(data_, world_->getConfigurationNumber());

    // Data begins
    data_ = set(data_, objList.size());

    for (auto *ob : objList) {

      // set gc
      if (ob->getObjectType() == ObjectType::ARTICULATED_SYSTEM) {
        data_ = set(data_, static_cast<ArticulatedSystem *>(ob)->getVisOb().size()+static_cast<ArticulatedSystem *>(ob)->getVisColOb().size());

        for(unsigned long int i=0; i < 2; i++) {
          std::vector <VisObject> *visOb;
          if (i == 0)
            visOb = &(static_cast<ArticulatedSystem *>(ob)->getVisOb());
          else
            visOb = &(static_cast<ArticulatedSystem *>(ob)->getVisColOb());

          for (int k=0; k<(*visOb).size(); k++) {
            auto& vob = (*visOb)[k];
            std::string name = std::to_string(ob->getIndexInWorld()) + separator() + std::to_string(i) + separator() + std::to_string(k);
            data_ = setString(data_, name);

            Vec<3> pos, offsetInWorld;
            Vec<4> quat;
            Mat<3, 3> bodyRotation, rot;

            ob->getPosition_W(vob.localIdx, pos);
            ob->getOrientation_W(vob.localIdx, bodyRotation);
            matvecmul(bodyRotation, vob.offset, offsetInWorld);
            matmul(bodyRotation, vob.rot, rot);
            data_ = set(data_, pos[0] + offsetInWorld[0]);
            data_ = set(data_, pos[1] + offsetInWorld[1]);
            data_ = set(data_, pos[2] + offsetInWorld[2]);

            raisim::rotMatToQuat(rot, quat);

            data_ = set(data_, quat[0]);
            data_ = set(data_, quat[1]);
            data_ = set(data_, quat[2]);
            data_ = set(data_, quat[3]);
          }
        }
      } else {
        data_ = set(data_, size_t(1));
        Vec<3> pos;
        Vec<4> quat;
        std::string name = std::to_string(ob->getIndexInWorld());
        data_ = setString(data_, name);
        dynamic_cast<SingleBodyObject *>(ob)->getPosition_W(pos);
        dynamic_cast<SingleBodyObject *>(ob)->getQuaternion(quat);
        data_ = setN(data_, pos.v, 3);
        data_ = setN(data_, quat.v, 4);
      }
    }
    updateReady_ = false;
  }

  void serializeObjects() {
    std::lock_guard<std::mutex> guard(worldMutex_);

    auto &objList = world_->getObjList();

    // set message type
    data_ = set(data_, ServerMessageType::INITIALIZATION);

    // set configuration number.
    data_ = set(data_, world_->getConfigurationNumber());

    // Data begins
    data_ = set(data_, objList.size());

    for (auto *ob : objList) {
      // set name length
      data_ = set(data_, ob->getIndexInWorld());

      //object type
      data_ = set(data_, ob->getObjectType());

      switch (ob->getObjectType()) {
        case SPHERE:
          data_ = set(data_, float(static_cast<Sphere *>(ob)->getRadius()));
          break;

        case BOX:
          for (int i = 0; i < 3; i++)
            data_ = set(data_, float(static_cast<Box *>(ob)->getDim()[i]));
          break;

        case CYLINDER:
          data_ = set(data_, float(static_cast<Cylinder *>(ob)->getRadius()));
          data_ = set(data_, float(static_cast<Cylinder *>(ob)->getHeight()));
          break;

        case CONE:
          break;

        case CAPSULE:
          data_ = set(data_, float(static_cast<Cylinder *>(ob)->getRadius()));
          data_ = set(data_, float(static_cast<Cylinder *>(ob)->getHeight()));
          break;

        case HALFSPACE:
          data_ = set(data_, float(static_cast<Capsule *>(ob)->getHeight()));
          break;

        case COMPOUND:
          break;

        case HEIGHTMAP:
          // misc data
          data_ = set(data_, float(static_cast<HeightMap *>(ob)->getCenterX()));
          data_ = set(data_, float(static_cast<HeightMap *>(ob)->getCenterY()));
          data_ = set(data_, float(static_cast<HeightMap *>(ob)->getXSize()));
          data_ = set(data_, float(static_cast<HeightMap *>(ob)->getYSize()));
          data_ = set(data_, static_cast<HeightMap *>(ob)->getXSamples());
          data_ = set(data_, static_cast<HeightMap *>(ob)->getYSamples());

          // size of height map
          data_ = set(data_, static_cast<HeightMap *>(ob)->getHeightVector().size());

          // height values in float
          for (auto h : static_cast<HeightMap *>(ob)->getHeightVector())
            data_ = set(data_, float(h));

          break;

        case ARTICULATED_SYSTEM:
          std::string resDir = static_cast<ArticulatedSystem *>(ob)->getResourceDir();
          data_ = setString(data_, resDir);

          for(unsigned long int i=0; i < 2; i++) {
            std::vector<VisObject>* visOb;
            if(i == 0)
              visOb = &(static_cast<ArticulatedSystem *>(ob)->getVisOb());
            else
              visOb = &(static_cast<ArticulatedSystem *>(ob)->getVisColOb());

            data_ = set(data_, visOb->size());

            for (auto& vob : *visOb) {
              data_ = set(data_, vob.shape);
              data_ = set(data_, i);
              if(vob.shape == Shape::Mesh) {
                data_ = setString(data_, vob.fileName);
                data_ = set(data_, vob.scale[0]);
                data_ = set(data_, vob.scale[1]);
                data_ = set(data_, vob.scale[2]);
              } else {
                data_ = set(data_, vob.visShapeParam.size());
                for(auto vparam: vob.visShapeParam)
                  data_ = set(data_, vparam);
              }
            }
          }
          break;
      }
    }
  }

  void changeRealTimeFactor() {
    get(&receive_buffer[0], &realTimeFactor);
    data_ = set(data_, ServerMessageType::NO_MESSAGE);
    return;
  }

  bool waitForReadEvent(int timeout) {
    fd_set sdset;
    struct timeval tv;

    tv.tv_sec = timeout;
    tv.tv_usec = 0;
    FD_ZERO(&sdset);
    FD_SET(server_fd, &sdset);
    if (select(server_fd+1, &sdset, NULL, NULL, &tv) > 0)
      return true;

    return false;
  }

  char *data_;
  World *world_;
  std::vector<char> receive_buffer, send_buffer;
  bool connected_ = false;
  char tempBuffer[MAXIMUM_PACKET_SIZE];
  int state_ = STATUS_RENDERING;

  bool pauseRequested_ = false;
  bool resumeRequested_ = false;
  double realTimeFactor = 1.0;
  std::atomic<bool> terminateRequested_ = {false};

  int client_;
  int server_fd, valread;
  sockaddr_in address;
  int addrlen;
  std::thread serverThread_;
  bool updateReady_ = true;

  std::mutex worldMutex_;

};

}

#endif //RAISIM_RAISIMSERVER_HPP
