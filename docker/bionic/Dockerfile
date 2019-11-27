FROM ubuntu:18.04

RUN apt-get update -qq && apt-get -y -qq upgrade && apt-get install -y -qq \
    build-essential \
    git \
    software-properties-common \
    cmake

# g++ 
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get install -y -qq g++-8 gcc-8

# eigen
RUN apt-get install -y -qq \
    libeigen3-dev