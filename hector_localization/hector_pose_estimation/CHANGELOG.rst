^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_pose_estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2021-02-16)
------------------
* Update maintainer email address
* Increase minimum CMake version to 3.0.2 to avoid the CMP0048 warning
  See
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
  for details.
* Merge pull request `#12 <https://github.com/tu-darmstadt-ros-pkg/hector_localization/issues/12>`_ from mikaelarguedas/patch-1
  update to use non deprecated pluginlib macro
* update to use non deprecated pluginlib macro
* Contributors: Johannes Meyer, Mikael Arguedas

0.3.0 (2016-06-27)
------------------
* hector_pose_estimation: removed trailing / in directory install rule
* Contributors: Johannes Meyer

0.2.2 (2016-06-24)
------------------
* hector_pose_estimation: cleaned up example launch file and add install rule for launch/ and rviz_cfg/
* update rviz file
* add .rviz config file for hector pose estimation
* add launch file for hector_pose_estimation
* Contributors: Johannes Meyer, libing64

0.2.1 (2015-11-08)
------------------

0.2.0 (2015-02-22)
------------------
* added QUIET flag when finding optional hector_timing
* immediately reset the gps measurement when fix is lost
* disabled publish_world_nav_transform parameter by default and added deprecation warnings for publish_world_map_transform and map_frame
* fixed sensor_pose correction if reference heading != 0
* added new publisher /geopose with type geographic_msgs/GeoPose
* instrumented code for timing measurements using hector_diagnostics
  See https://github.com/tu-darmstadt-ros-pkg/hector_diagnostics/tree/master/hector_timing.
* added raw, unfiltered sensor pose publisher
  This is for debugging purposes only. The raw sensor pose combines the information from
  IMU acceleration/gravity, magnetic, GPS and height/barometric pressure sensors.
* added world to nav transform broadcaster and global/reference (geographic_msgs/GeoPose) publisher
* Added missing orientation-orientation derivative in GenericQuaternionSystemModel
* Readded dimension paramters to SystemModel templates
* Readded class BaseState and State::base() accessors (currently unused)
* Added typed getters PoseEstimation::getSystem_<T>(name) and PoseEstimation::getMeasurement_<T>(name)
* IMU-based prediction is now in the GenericQuaternionSystemModel class again.
  The accelerometer and gyro models are added automatically if GenericQuaternionSystemModel::init() found an IMU input.
  GenericQuaternionSystemModel calls methods on the IMU model classes to get the errors, derivatives and noise.
  The IMU models themselves do not add
* Added class SkewSymmetricMatrix
* Avoid calling SymmetricMatrix::symmetric() as, surprisingly, this method seems to be quite expensive. Symmetry is only
  enforced if assigned from a matrix which is not a SymmetricMatrix.
* Added Eigen optimizations (avoid copies and added alignment macros)
* Moved system and measurement status management from class Filter to PoseEstimation
* first functional version with multiplicative EKF
* added support for dynamic states (full, orientation only, position/velocity only, ...)
* Contributors: Johannes Meyer

0.1.5 (2014-10-02)
------------------
* hector_pose_estimation: make private variables protected to allow access from specialized pose_estimation classes
* hector_pose_estimation: fixed wrong indicies bug in GPS pose publisher
* Contributors: Johannes Meyer

0.1.4 (2014-08-28)
------------------

0.1.3 (2014-07-09)
------------------

0.1.2 (2014-06-02)
------------------

0.1.1 (2014-03-30)
------------------
* Fixed boost 1.53 issues
  changed boost::shared_dynamic_cast to boost::dynamic_pointer_cast and
  boost::shared_static_cast to boost::static_pointer_cast
* readded tf_prefix support (was removed from tf::TransformBroadcaster in hydro)
* hector_pose_estimation/rtt_hector_pose_estimation: removed optional usage of package hector_uav_msgs and created a separate package hector_quadrotor_pose_estimation instead
* fixed unresolved symbols when loading the nodelet, renamed plugin description xml file
* Contributors: Christopher Hrabia, Johannes Meyer

0.1.0 (2013-09-03)
------------------
* catkinized stack hector_localization
