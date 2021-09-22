^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_components_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2016-06-24)
------------------
* fixed for checkerboard
* Add checkerboard with associated macro.
* Added calibration and fixed an origin bug at the spinnning joint of the lidar
* Added realistic inertias and masses. Moved RGB-D Cam according to reality
* Contributors: Marius Schnaubelt, Martin Oehler, Stefan Kohlbrecher

0.4.1 (2015-11-08)
------------------
* hector_components_description/hector_sensors_description: added xacro namespace prefix to macro calls
* Cleaned up root element xmlns arguments according to http://gazebosim.org/tutorials?tut=ros_urdf#HeaderofaURDFFile
* Added missing xacro namespace prefix to XML tags
* Contributors: Johannes Meyer

0.4.0 (2015-11-07)
------------------
* Renamed LIDAR and RGBD cam for thor compatibility
* Remove gazebo tags for links without visuals
* First version of the new head, the hector multisensor head
* Add addons xacro files
* Update how spinning hokuyo is set up
* Update spinning lidar mount properties
* Fix stupid mixup of min and max lidar angle
* Reduce spinning lidar mount mass
* Fix parent not used correctly in spinning lidar mount
* Update rotating hokuyo transform
* Refactor spinning lidar mount
* Update LIDAR mount with reasonable inertia
* URDF hardware interface changes for new gazebo ros control style (#185)
* Add second spinning hokuyo variant
* Fix origin block not getting used correctly
* Add gazebo_ros_control required tags to spinning lidar macros
* Formatting
* Add spinning lidar mount and hokuyo example
* Contributors: Marius Schnaubelt, Stefan Kohlbrecher

0.3.2 (2014-09-01)
------------------
* increased maximum torque for camera servos in vision_box_common.gazebo.xacro
* adapted urdf for asus xtion and added camera variables
* Add simple ps eye geometry
* Contributors: Johannes Meyer, Stefan Kohlbrecher

0.3.1 (2014-03-30)
------------------
* Re-parent LIDAR and camera mount to top_box_link
* Add xacro macros for setting dimensions
* Remove obsolete files
* Add UTM-30LX macro to vision box xacro
* Add hector ugv vision box to hector_components_description package for better reusability
* Contributors: Stefan Kohlbrecher
