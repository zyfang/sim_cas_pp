# sim_cas_pp catkin package for post processing data from sim_cas

## Installation

### Prerequisites:
Get MongoDB: http://docs.mongodb.org/manual/tutorial/install-mongodb-on-ubuntu/

Get C++ Client for MongoDB (branch ```26compat```): https://github.com/mongodb/mongo-cxx-driver/wiki/Download-and-Compile-the-Legacy-Driver

### Configuration:
Make sure to set the path the installation of the C++ Client for MongoDB in the ```CMakeLists.txt```!

### Run example:
~~~
gazebo -u --verbose -s libPostProcess.so -p <path>/state.log --db mydb --collection mycol --suffix 2 --delay 2
~~~
- --db: name of the database in which the data is stored
- --collection: name of the collections in which the data is stored (type of collection is concatenated to the name, e.g. mycol_tf)
- --suffix: number to be concatenated to collectionname and multiplier of the timeoffset. This is to accommodate storing multiple episodes in the openEASE database
- --delay: at which simtime postprocessing should start in seconds, will not start before the given time
- -replaying: set flag if postprocessing is done using logs, rather than by connecting to the live play


