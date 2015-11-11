# sim_games_pp #

Catkin package for post processing data from [sim_games](https://bitbucket.org/ahaidu/sim_games).

### Prerequisites ###

 * [Gazebo](http://gazebosim.org). 
 * ROS
 * MongoDB
 * [libconfig](http://www.hyperrealm.com/libconfig/)

### Usage ###
~~~
gazebo -u --verbose -s libPostProcess.so -p <path>/state.log --suffix 69
~~~