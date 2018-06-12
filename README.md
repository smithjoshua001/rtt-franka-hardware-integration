# rtt-franka-hardware-integration
Integration of Franka Panda arms into Orocos

## Installation
Besides an Orocos toolchain, this project requires [libfranka](https://github.com/frankaemika/libfranka) built from source for compilation. With <libfranka> as your location of the libfrankasources, use

    mkdir build && cd build
    cmake -DFranka_DIR=<libfranka>/build ..
    cmake --build .
    
to build an Orocos component from the code.
