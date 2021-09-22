^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_sensors_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.2 (2016-06-24)
------------------
* Update flir a35 camera macro
* Add gazebo material for flir and realsense models
* Add models for flir a35 and realsense r200 cameras
* Formatting of thermaleye_camera macro
* Contributors: Stefan Kohlbrecher, kohlbrecher

0.4.1 (2015-11-08)
------------------
* hector_components_description/hector_sensors_description: added xacro namespace prefix to macro calls
* Cleaned up root element xmlns arguments according to http://gazebosim.org/tutorials?tut=ros_urdf#HeaderofaURDFFile
* hector_sensors_description: removed deprecated plugin parameters and added noise to the hokuyo_utm30lx_model macro (fix #1)
* Contributors: Johannes Meyer

0.4.0 (2015-11-07)
------------------
* Add zoom camera xacro macro. Only works starting with Gazebo6
* Update asus_camera.urdf.xacro
  Clarify macro use.
* Remove link geometries where not needed
  Add generic_thermal_camera macro
* Update how spinning hokuyo is set up
* Update hokuyo gpu xacro macro
* Properly use camera name
* changed asus description, collision geometry needs to match visual geometry for 3d self filter to work.
* Add generic stereo camera macro
* Use cylinder collision geom as box gives spurious errors in LIDAR scans in some URDFs
* Contributors: Florian Kunz, Stefan Kohlbrecher

0.3.2 (2014-09-01)
------------------
* Updated asus xtion pro live mesh to reflect actual sensor dimensions, add stl version
* Contributors: Stefan Kohlbrecher

0.3.1 (2014-03-30)
------------------
* added hokuyo_utm30lx_model and hokuyo_utm30lx_gpu macros and disabled gpu laser in default hokuyo_utm30lx macro
* use gpu_ray sensor in hydro
* Contributors: Johannes Meyer

0.3.0 (2013-09-02)
------------------
* catkinized stack hector_models
