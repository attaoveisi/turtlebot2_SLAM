# turtlebot2_SLAM
This is an example of how to to integrate a fast SLAM algorithm with Turtlebot2. For this purposes, the slam_gmapping package is used.

![image](https://user-images.githubusercontent.com/17289954/103134730-8a669100-46b3-11eb-8ac3-8faf40f6c147.png)

The robot is simulated in the wollowgarage world in turtlebot_gazebo package.

## SLAM

SLAM is the problem of estimating a map and robot pose given the measurements from environment and inputs:

![image](https://user-images.githubusercontent.com/17289954/103096067-01901c80-4603-11eb-871d-ad83134d9b4e.png)

There are two types of SLAM: Online-SLAM and Full-SLAM

### Online SLAM

In online SLAM the robot uses the measurements and control information from the newest measurement, neglecting anything older than that:

![image](https://user-images.githubusercontent.com/17289954/103096136-40be6d80-4603-11eb-92d6-bcb10ab4ffc5.png)

thus we may write it as *p(x_t,m|z_1:t,u_1:t)*

### Full SLAM

In Full SLAM problem, the ful information is used in order to estimate the map and pose:


![image](https://user-images.githubusercontent.com/17289954/103096257-a14daa80-4603-11eb-961e-0856dfabde00.png)

thus we may write it as *p(x_1:t,m|z_1:t,u_1:t)*. One can define the relationship between these two by integrating Full-SLAM posterior over all of its poses:

![image](https://user-images.githubusercontent.com/17289954/103096366-fab5d980-4603-11eb-9d37-c59713369be5.png)

Regardless of Full or Online SLAM, SLAM can be both continuous or discrete. During SLAM, a robot continuously collects odometry information to estimate the robot poses and continuously senses the environment to estimate the location of the object or landmark. Thus, both robots poses and object location are continuous aspects of the SLAM problem.


robots continuously sense the environment to estimate the location of the objects, when doing so SLAM algorithms have to identify if a relation exists between any newly detected objects and previously detected ones. This helps the robot understand if it has been in this same location before. At each moment, the robot has to answer the question, “Have I been here before?”. The answer to this question is binary - either yes or no - and that’s what makes the relation between objects a discrete component of the SLAM problem.This discrete relation between objects is known by correspondence. 


Both type of SLAMs include the *corespondence* in their posterior definitions as *p(x_1:t,c_1:t,m|z_1:t,u_1:t)* and  *p(x_t,c_t,m|z_1:t,u_1:t)* for full and online versions, respectively.

### Challenges

The continuous parameter space composed of the robot poses and the location of the objects is highly dimensional. While mapping the environment and localizing itself, the robot will encounter many objects and have to keep track of each one of them. Thus, the number of variables will increase with time, and this makes the problem highly dimensional and challenging to compute the posterior. 

The discrete parameter space is composed out of the correspondence values, and is also highly dimensional due to the large number of correspondence variables. Not only that, the correspondence values increase exponentially over time since the robot will keep sensing the environment and relating the newly detected objects to the previously detected ones.


## Fast SLAM

The FastSLAM algorithm solves the Full SLAM problem with known correspondences.

**Estimating the Trajectory:** FastSLAM estimates a posterior over the trajectory using a particle filter approach. This will give an advantage to SLAM to solve the problem of mapping with known poses.
**Estimating the Map:** FastSLAM uses a low dimensional Extended Kalman Filter to solve independent features of the map which are modeled with local Gaussian.

The custom approach of representing the posterior with particle filter and Gaussian is known by the Rao-Blackwellized particle filter approach. FastSLAM algorithm can solve the full SLAM problem with known correspondences. 

three different instances of the FastSLAM algorithm exist.

### FastSLAM 1.0

The FastSLAM 1.0 algorithm is simple and easy to implement, but this algorithm is known to be inefficient since particle filters generate sample inefficiency.

### FastSLAM 2.0

The FastSLAM 2.0 algorithm overcomes the inefficiency of FastSLAM 1.0 by imposing a different distribution, which results in a low number of particles. Keep in mind that both of the FastSLAM 1.0 and 2.0 algorithms use a low dimensional Extended Kalman filter to estimate the posterior over the map features.


The main advantage of the FastSLAM 1.0 and 2.0 algorithms is that it uses a particle filter approach to solve the SLAM problem. Each particle will hold a guess of the robot trajectory, and by doing so, the SLAM problem is reduced to mapping with known poses. But, in fact, this algorithm presents a big disadvantage since it must always assume that there are known landmark positions, and thus with FastSLAM we are not able to model an arbitrary environment.

### Grid-based FastSLAM


The third instance of FastSLAM is really an extension to FastSLAM known as the grid-based FastSLAM algorithm, which adapts FastSLAM to grid maps. 

With the grid mapping algorithm one can model the environment using grid maps without predefining any landmark position. So by extending the FastSLAM algorithm to occupancy grid maps, one can solve the SLAM problem in an arbitrary environment. 


![image](https://user-images.githubusercontent.com/17289954/103097095-6d27b900-4606-11eb-900b-a966e921b1f1.png)

the grid-based FastSLAM consists of thre steps:

![image](https://user-images.githubusercontent.com/17289954/103132599-8df31b80-46a5-11eb-96dc-cbcdc5d5bb7f.png)

The algorithm procedure is explained as

![image](https://user-images.githubusercontent.com/17289954/103132644-e0ccd300-46a5-11eb-815e-c3c65b3ebc4e.png)

## How to run the package?

clone the package `git clone https://github.com/attaoveisi/turtlebot2_SLAM.git`

build and source `cd turtlebot2_SLAM/workspace/catkin_ws && catkin_make && surce devel/setup.bash`

Deploy the Turltebot in the Willow Garage environment:

`$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=worlds/willowgarage.world`

![image](https://user-images.githubusercontent.com/17289954/103141614-0a621a80-46f7-11eb-9906-3bdde1cf5522.png)

For SLAM we are using slma_gmapping package (see [link](http://wiki.ros.org/slam_gmapping)) which subscribes to `tf` which should contain info about the relation of laser, base, and odometry frames as well as the laser topic (`sensor:msgs/LaserScan`) and it publishe the map as `nav_msgs/OccupancyGrid` and `std_msgs/Float64`.

the turtlebot will be driven with teleop package (keyboard). We will fake the laser measurement as it can be seen in `/workspace/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch`:

```sh
  <!-- Fake laser -->
  <node pkg="nodelet" type="nodelet" name="laserscan_nodelet_manager" args="manager"/>
  <node pkg="nodelet" type="nodelet" name="depthimage_to_laserscan"
        args="load depthimage_to_laserscan/DepthImageToLaserScanNodelet laserscan_nodelet_manager">
    <param name="scan_height" value="10"/>
    <param name="output_frame_id" value="camera_depth_frame"/>
    <param name="range_min" value="0.45"/>
    <remap from="image" to="/camera/depth/image_raw"/>
    <remap from="scan" to="/scan"/>
  </node>
```

in a second terminal, launch the turtlebot2 teleop package by: 

```
$ cd /home/workspace/catkin_ws
$ source devel/setup.bash
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```

![image](https://media.giphy.com/media/hj1HL3XpHSTPPLSrlt/giphy.gif)


in a third terminal run the gmapping node as:

```
$ cd /home/workspace/catkin_ws
$ source devel/setup.bash
$ rosrun gmapping slam_gmapping
```


in a fourth terminal `rosrun rviz rviz -d /turtlebot2_SLAM/workspace/catkin_ws/c/gmapping.rviz` and apply the rviz config

finally, drive the robot around and if you are satisfied with the map, save the map in a fifthe terminal with `rosrun map_server map_saver -f myMap`


![image](https://media.giphy.com/media/EZXEp4dufLPbiJmedV/giphy.gif)


With the map_server you can load and save maps. Running map_server will generate the map.pgm and the map.yaml files:


1- map.pgm: Picture of the map in occupancy grid representation

    White pixels: Free cells
    Black pixels: Occupied cells
    Gray pixels: Unknown state


2- map.yaml: The map metadata

    image: Map name
    resolution: Resolution of the map (meters/pixel)
    origin: Pose of the lower-left pixel in the map (x, y, Θ)
    Occupied_thresh: Cell is considered occupied if its probability is greater than this threshold.
    free_thresh: Cell is considered unoccupied or free if its probability is less than this threshold.
    negate: This value will check whether the notation of black colored cell=occupied and white colored cell = free should be preserved


If the map quality is low, it is because the gmapping parameters values used were the default values. In general, it’s essential to tune them in order to get a 100% accurate map. These parameters are all listed under the gmapping documentation, where you can look at them yourself. If you experiment with some of these parameter values, you should be able to get better maps.

For example, you might try, reducing the angularUpdate and linearUpdate values so the map gets updated for smaller ranges of movements, reducing the x and y limits, which represent the initial map size, increasing the number of particles. You can try tweaking these parameters and/or any other parameter you think should be changed. 





