# turtlebot2_SLAM
This is an example of how to to integrate a fast SLAM algorithm with Turtlebot2

## The SLAM is the problem of estimating a map and robot pose given the measurements from environment and inputs:

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

![image](https://user-images.githubusercontent.com/17289954/103097162-a8c28300-4606-11eb-9ca8-c26ea85199c1.png)

![image](https://user-images.githubusercontent.com/17289954/103132599-8df31b80-46a5-11eb-96dc-cbcdc5d5bb7f.png)




