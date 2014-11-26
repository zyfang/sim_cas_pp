cmake_minimum_required(VERSION 2.8.3)
project(kitchen_games_pp)

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

## Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  find_package(GAZEBO REQUIRED)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MONGO_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
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