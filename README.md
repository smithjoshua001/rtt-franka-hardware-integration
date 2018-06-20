# rtt-franka-hardware-integration
Integration of Franka Panda arms into Orocos

## Installation
Besides an Orocos toolchain, this project requires _ [libfranka](https://github.com/frankaemika/libfranka) _ built from source for compilation. With <libfranka> as your location of the _libfranka_ sources, use

    mkdir build && cd build
    cmake -DFranka_DIR=<libfranka>/build ..
    cmake --build .
    
to build an Orocos component from the code.
