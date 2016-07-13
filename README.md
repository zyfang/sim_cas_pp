# sim_cas_pp catkin package for post processing data from sim_cas

### Prerequisites:
- Get MongoDB: http://docs.mongodb.org/manual/tutorial/install-mongodb-on-ubuntu/
- Get C++ Client for MongoDB (branch ```26compat```): https://github.com/mongodb/mongo-cxx-driver/wiki/Download-and-Compile-the-Legacy-Driver

### Configuration:
- Make sure to set the path the installation of the C++ Client for MongoDB in the ```CMakeLists.txt```!

- A range of options is available in config.cfg:
	- *mongo*: 
		- "port" sets the mongodb port the data will be written to
		- "db_name" and "coll_name" are the default values that will be used if they are not set in the call (see Run example)
	- *LogTF*:
		Set variables that control the logging behavior of the logger. Set the logging thresholds (changes below the threshold will not be recorded) and whether it should publish to a topic. 
	- *LogRaw*:
		Set whether want to apply logging thresholds and if so, what they should be.
	- *LogEvents*:
		Identify which collisions are of which type (e.g. supporting faces vs tools) and how long events should be before they're concatenated (to get rid of "flickering").
	- *LogMotionExpression*:
		Which file to use to extract the motion constraints/expressions that should be recorded and indicate which is the controlled and which the observed (target) model.


### Run example:
*Make sure to have a running mongo server. For example: `mongod --port 27018 --dbpath /home/test/mymongo/var/lib/mongodb/`*

~~~
gazebo -u --verbose -s libPostProcess.so -p <path>/state.log --db mydb --collection mycol --suffix 2 --delay 2
~~~
- --db: name of the database in which the data is stored
- --collection: name of the collections in which the data is stored (type of collection is concatenated to the name, e.g. mycol_tf)
- --suffix: number to be concatenated to collectionname and multiplier of the timeoffset. This is to accommodate storing multiple episodes in the openEASE database
- --delay: at which simtime postprocessing should start in seconds, will not start before the given time
- -replaying: set flag if postprocessing is done using logs, rather than by connecting to the live play


