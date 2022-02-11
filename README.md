# hector_quadrotor ported to ROS Noetic & Gazebo 11

<img src="imgs/dron_photo.png" height="250"/> <img src="imgs/dron_photo_rviz.png" height="250"/>

***.:: First version, please tell me the issues or help me to fix it ::.***

<< I take part of this from __The Construct's__ [repo](https://bitbucket.org/theconstructcore/hector_quadrotor_sim/src/master/) and YouTube [chanel](https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q) >>.

## Requirements

__1__ You need the following packages before install hector_quadrotor_noetic.

* unique_identifier (Melodic version works):
    ```sh
    git clone https://github.com/ros-geographic-info/unique_identifier.git
    ```
* geographic_info:
    ```sh
    git clone https://github.com/ros-geographic-info/geographic_info
    ```

__2|__ Build.
```sh
cd ~/catkin_ws && catkin_make
```

3| Clone hector_quadrotor_noetic.
```sh
git clone https://github.com/RAFALAMAO/hector_quadrotor_noetic.git
```

4| Repeat step 2.

## Usage

Run a simulation by executing the launch file in `hector_quadrotor_gazebo` and `hector_quadrotor_demo` packages (only this works at the momment, but you can try the other ones):

* `roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch`
* `roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch`
* `roslaunch hector_quadrotor_demo outdoor_flight_gazebo_no_rviz.launch`

You can control it with teleop_twist_keyboard.
* `git clone https://github.com/ros-teleop/teleop_twist_keyboard`

Here is a [video](https://www.youtube.com/watch?v=-2IWfZjqoNc) testing it:

<img src="imgs/gif.GIF"/>
