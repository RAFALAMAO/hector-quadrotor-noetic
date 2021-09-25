## hector_quadrotor ported to ROS Noetic with Gazebo 11

!First version, please tell me the issues!

I take part of this from:`https://bitbucket.org/theconstructcore/hector_quadrotor_sim/src/master/`

1. You need to have this packages before install hector_quadrotor_noetic.
    
    * unique_identifier (Melodic version works):
    `git clone https://github.com/ros-geographic-info/unique_identifier.git`

    * geographic_info:
    `git clone https://github.com/ros-geographic-info/geographic_info`
    
2. Buid
    * `cd ~/catkin_ws`
    * `catkin_make`

3. Run the simulation
Run a simulation by executing a launch file in cvg_sim_gazebo package: 

* `roslaunch cvg_sim_gazebo ardrone_testworld.launch`

:D
