This is the first iteration, which is Python-based. Plans for future iterations involve moving to C++ to allow for custom message files in a ROS2 package.

Note that an issue that we ran into was that the LiDAR would stop sending data after a certain amount of data points, so the main files (sensorFusion*.py) have a hard-coded counter at which the LiDAR object will be stopped and recreated. Time did not allow for proper troubleshooting of the bug, or for the files to be translated to a C++-based ROS2 package.
