cmake_minimum_required(VERSION 2.8.3)
project(sim_games_pp)

## TODO manually added
set(MONGOCLIENT /home/haidu/bin/mongo-client-install/lib/libmongoclient.so)
set(MONGO_INCLUDE_DIR /home/haidu/bin/mongo-client-install/include)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  beliefstate_client
)

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
  src/LogRaw.cc
)

target_link_libraries(PostProcess
  ${catkin_LIBRARIES}
  ${MONGOCLIENT}
  ${GAZEBO_LIBRARIES}
  config++
)
