cmake_minimum_required(VERSION 2.8.3)
project(rtt-franka-hardware-integration)

#set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g -Wall -fPIC -DHAVE_SOCKLEN_T")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )


set ( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/Modules/  )
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(urdfdom_headers 0.4.0 REQUIRED)
message(STATUS "urdfdom_headers: ${urdfdom_headers_INCLUDE_DIRS} ${urdfdom_headers_LIBRARIES}")
find_package(srdfdom_advr REQUIRED)
find_library(srdfdom_advr_LIBRARY ${srdfdom_advr_LIBRARIES} HINTS ${srdfdom_advr_LIBRARY_DIRS})
find_package(XBotCoreModel REQUIRED)
message(STATUS "XBotCoreModel: ${XBotCoreModel_INCLUDE_DIRS} ${XBotCoreModel_LIBRARIES}")
find_library(XBotCoreModel_LIBRARY ${XBotCoreModel_LIBRARIES} HINTS ${XBotCoreModel_LIBRARY_DIRS})
message(STATUS "XBotCoreModel: ${XBotCoreModel_INCLUDE_DIRS} ${XBotCoreModel_LIBRARY}")

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  message(STATUS "checking for xenomai")
  find_package(Xenomai REQUIRED)
  message(STATUS ${Xenomai_LIBRARY_DIRS})
  set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS XENOMAI)
#  message(STATUS "checking for RTNET")
#  find_package(RTnet)
#  if(${RTNET_FOUND})
#    set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET)
#    set(RTLIB_LIBRARIES rtdm)
#    include_directories(${RTNET_INCLUDE_DIR})
#  endif()
endif()


find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

# find_package(PkgConfig REQUIRED)
# pkg_search_module(TINYXML REQUIRED tinyxml)
# if(NOT TINYXML_INCLUDE_DIR)
#   find_path("/usr/include" tinyxml.h)
#   find_library(TINYXML_LIBRARY NAMES tinyxml)
#   set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
#   set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
#   set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
# endif()

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML2 REQUIRED TinyXML2 tinyxml2)
message(STATUS "HAS FOUND? ${TINYXML2_FOUND}")
message(STATUS "LIBRARYS: ${TINYXML2_LIBRARIES}")
find_library(TinyXML_LIBRARIES ${TINYXML2_LIBRARIES} HINTS ${TINYXML2_LIBRARY_DIRS})
#ExtractLibraries(TinyXML_LIBRARIES ${TINYXML2_LIBRARY_DIRS} ${TINYXML2_LIBRARIES})
message(STATUS "TINYXML2 LIBS: ${TinyXML_LIBRARIES}")

#include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
#link_directories(${TINYXML_LIBRARY_DIR})

find_package(urdfdom REQUIRED)
include_directories(BEFORE SYSTEM ${urdfdom_INCLUDE_DIRS})
link_directories(BEFORE SYSTEM ${urdfdom_LIBRARY_DIR})
message(STATUS "URDFDOM: ${urdfdom_INCLUDE_DIRS}")
message(STATUS "URDFDOM: ${urdfdom_LIBRARIES}")

find_package(orocos_kdl REQUIRED)
include_directories(BEFORE SYSTEM ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

find_package(Franka REQUIRED)
get_target_property(Franka_INCLUDE_DIRS ${Franka_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${Franka_INCLUDE_DIRS}
    ${Franka_DIR}/../src
    ${Franka_DIR}/../common/include
)
message(STATUS "FRANKA: ${Franka_INCLUDE_DIRS}")

find_library(Poco REQUIRED COMPONENTS Net Foundation)
message(STATUS "POCO: ${Poco_LIBRARY}")

link_directories(
  ${orocos_kdl_LIBRARY_DIRS}
  ${PROJECT_NAME_BIN}/lib
  ${Boost_LIBRARY_DIR}
  ${Xenomai_LIBRARY_DIRS}
  ${Franka_LIBRARY_DIRS}
)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${OROCOS-RTT_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  ${RST-RT_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${srdfdom_advr_INCLUDE_DIRS}
  ${XBotCoreModel_INCLUDE_DIRS}
)

# ADD_LIBRARY(FRANKA_RTT
#
# )

orocos_component(rtt_franka_hw src/rtt-franka-robot_hooks.cpp src/rtt-franka-robot.cpp src/kinematic_chain.cpp)
set_target_properties(rtt_franka_hw PROPERTIES COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(rtt_franka_hw
  ${orocos_kdl_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES}
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY}
  ${OROCOS-RTT_TYPEKIT_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
  ${RSC_LIBRARIES}
  ${RCI_LIBRARIES}
  ${RTLIB_LIBRARIES}
  ${RST-RT_LIBRARIES}
  ${TINYXML_LIBRARIES}
  ${urdfdom_LIBRARIES}
  ${urdfdom_headers_LIBRARIES}
  ${srdfdom_advr_LIBRARY}
  ${XBotCoreModel_LIBRARY}
  ${Franka_LIBRARIES}
  PocoFoundation
  PocoNet
)

orocos_generate_package(INCLUDE_DIRS include)

message(STATUS "Generarating ${CMAKE_SOURCE_DIR}/.clang_complete")
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
file(WRITE ${CMAKE_SOURCE_DIR}/.clang_complete "")
foreach(dir ${dirs})
  file(APPEND ${CMAKE_SOURCE_DIR}/.clang_complete "-I${dir}\n")
endforeach()

message(STATUS ${CMAKE_CXX_FLAGS})
# if(${CMAKE_CXX_FLAGS})
    message(STATUS ${CMAKE_CXX_FLAGS})
string(REPLACE "'" "" CMAKE_CXX_FLAGS_SPLIT ${CMAKE_CXX_FLAGS})
string(REPLACE " " ";" CMAKE_CXX_FLAGS_SPLIT ${CMAKE_CXX_FLAGS_SPLIT})
# endif()
foreach(flag ${CMAKE_CXX_FLAGS_SPLIT})
  file(APPEND ${CMAKE_SOURCE_DIR}/.clang_complete "${flag}\n")
endforeach()
