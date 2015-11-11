# sim_cas_pp catkin package for post processing data from sim_cas

## Installation

### Prerequisites:
Get MongoDB: http://docs.mongodb.org/manual/tutorial/install-mongodb-on-ubuntu/

Get C++ Client for MongoDB (branch ```26compat```): https://github.com/mongodb/mongo-cxx-driver/wiki/Download-and-Compile-the-Legacy-Driver

### Configuration:
Make sure to set the path the installation of the C++ Client for MongoDB in the ```CMakeLists.txt```!

### Run example:
~~~
gazebo -u --verbose -s libPostProcess.so -p <path>/state.log --db mydb --collection mycol
~~~

