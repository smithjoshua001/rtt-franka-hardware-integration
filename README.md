# rtt-franka-hardware-integration
Integration of Franka Panda arms into Orocos

## Installation
Besides an Orocos toolchain, this project requires [libfranka](https://github.com/frankaemika/libfranka) built from source for compilation. Since internals of libfranka are used from within the Orocos component, make sure to set the required include and library paths:

    export CPATH=<libfranka>/src:<libfranka>/common/include
    export LIBRARY_PATH=<libfranka>/build
    
where `<libfranka>` specifies the path of your libfranka source directory.
For CMake to find your libfranka build, set

    export Franka_DIR=<libfranka>/build
    
You should then be able to do the standard

    mkdir build && cd build
    cmake ..
    cmake --build .
    
to build an Orocos component from the code.
