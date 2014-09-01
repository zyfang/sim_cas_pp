# hand_sim catkin package for post processing data

### When adding the contacts to MongoDB start running it in pause mode, so initial values are not lost due to loading time.

### Run example:
~~~
gazebo -u -s libPostProcess.so -p <path>/state.log --verbose -db db_name.col_name -w world_name
~~~

