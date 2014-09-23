cmake_minimum_required(VERSION 2.8.3)
project(hand_sim_pp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  beliefstate_client
)

## TODO manually added
#find_package(Boost COMPONENTS chrono)
set(Boost_CHRONO_LIBRARY /usr/lib/x86_64-linux-gnu/libboost_chrono.so)
message(Chrono boost lib: "${Boost_CHRONO_LIBRARY}")

## TODO manually added
set(MONGOCLIENT /home/haidu/bin/mongo-client-install/lib/libmongoclient.so)
set(MONGO_INCLUDE_DIR /home/haidu/bin/mongo-client-install/include)

## Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  find_package(GAZEBO REQUIRED)
  pkg_check_modules(OGRE OGRE)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})

pkg_check_modules(OGRE-Terrain OGRE-Terrain)
include_directories(${OGRE-Terrain_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES beliefstate_client
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs tf
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MONGO_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
)


add_library(PostProcess src/PostProcess.cc)

target_link_libraries(PostProcess ${catkin_LIBRARIES} ${MONGOCLIENT})# ${Boost_CHRONO_LIBRARY})





