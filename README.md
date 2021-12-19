## hector_quadrotor ported to ROS Noetic with Gazebo 11

***# First version, please tell me the issues or help me to fix it #***

I take part of this from:`https://bitbucket.org/theconstructcore/hector_quadrotor_sim/src/master/`

1. You need to have this packages before install hector_quadrotor_noetic.
* unique_identifier (Melodic version works):
`git clone https://github.com/ros-geographic-info/unique_identifier.git`
* geographic_info:
`git clone https://github.com/ros-geographic-info/geographic_info`

2. Build
* `cd ~/catkin_ws`
* `catkin_make`

3. Then clone hector_quadrotor_noetic.
* `git clone https://github.com/RAFALAMAO/hector_quadrotor_noetic.git`

4. Repeat step 2

5. Run a simulation by executing a launch file in "hector_quadrotor_gazebo" and "hector_quadrotor_demo" package (only this works at the momment, but you can try the other ones):

* `roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch`
* `roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch`

6. You can control it with teleop_twist_keyboard
* `git clone https://github.com/ros-teleop/teleop_twist_keyboard`

You can watch a video testing it:
https://www.youtube.com/watch?v=-2IWfZjqoNc

😊😊😊
