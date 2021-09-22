# hector_localization
The hector_localization stack is a collection of packages, that provide the full 6DOF pose of a robot or platform.

Team Hector Darmstadt uses the stack to estimate the full 6D pose of the robot within the real-time loop of the Hector quadrotor and even for estimating the position, velocity and attitude of a small airplane as part of our flight mechanics lab.

The design goals have been similar to what Chad Rockey described in his answer. 

The core package currently provides a system model for generic 6DOF rigid body pose estimation based on pure IMU inputs (rates and accelerations), which can be specialized depending on the robot and for additional input values. As measurement models the package currently provides direct and barometric height measurements, GPS postion and velocity, magnetic field sensors as heading reference and a generic pose and twist measurement to fuse nav_msgs/Odometry messages from arbitrary sources (e.g. wheel odometry or SLAM).

The stack consists of a core library without dependencies to ROS beside message types and additional packages providing a node, nodelet or Orocos Real-Time Toolkit TaskContext interface. The system and measurement models could also be implemented and loaded as plugins with some minor modifications. In the background an Extended Kalman Filter based on the Bayesian Filter Library is responsible for fusing all information into a consistent state estimate and additionally keeps track which state variables are observable and which are not.


# How to run the code
    cd catkin_ws
    git clone git@github.com:tu-darmstadt-ros-pkg/hector_localization.git
    catkin_make --source hector_localization
    roslaunch hector_pose_estimation hector_pose_estimation.launch 

#rosbag for test
1. download the rosbag

    https://drive.google.com/folderview?id=0B4hFvojO5r3scWJRVWdhSmdLd0k&usp=sharing
    
2. replay the rosbag

    rosbag play 2016-03-09-22-11-07.bag
    
![hector_pose_estimation](https://cloud.githubusercontent.com/assets/3192355/14065311/5b968b2c-f457-11e5-862a-2f42720035b8.png)