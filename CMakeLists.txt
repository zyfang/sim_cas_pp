cmake_minimum_required(VERSION 2.8.3)
project(sim_cas_pp)

## TODO manually added
set(MONGOCLIENTPATH /usr/local)
set(MONGOCLIENT_LIBRARIES ${MONGOCLIENTPATH}/lib/libmongoclient.so)
set(MONGO_INCLUDE_DIR ${MONGOCLIENTPATH}/include)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  giskard
  beliefstate_client
)

find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/node.h PATH_SUFFIXES yaml-cpp )
find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp )

catkin_package()

## Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  find_package(GAZEBO REQUIRED)
  ###pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()


find_package (Qt4)

find_package(Protobuf REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${MONGO_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
)

add_library(
  PostProcess SHARED 
  src/PostProcess.cc 
  src/GzEvent.cc 
  src/GzEventObj.cc 
  src/LogTF.cc
  src/LogEvents.cc
  src/LogMotionExpressions.cc
  src/LogRaw.cc
  src/LogParticles.cc
)

target_link_libraries(PostProcess
  ${catkin_LIBRARIES}
  ${MONGOCLIENT_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  config++
  ${YAML_CPP_LIBRARIES}
)
