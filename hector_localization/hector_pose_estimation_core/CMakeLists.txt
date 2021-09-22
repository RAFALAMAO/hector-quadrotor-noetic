cmake_minimum_required(VERSION 3.0.2)
project(hector_pose_estimation_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rostime rosconsole roscpp sensor_msgs geometry_msgs nav_msgs geographic_msgs tf)

# Find Eigen3 headers
# see http://wiki.ros.org/jade/Migration#Eigen_CMake_Module_in_cmake_modules
# and https://github.com/ros/cmake_modules/issues/35
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_FOUND ${EIGEN_FOUND})
  set(EIGEN3_INCLUDE_DIR ${EIGEN_INCLUDE_DIRS})
  # set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  # Possibly map additional variables to the EIGEN3_ prefix.
endif()

# Find hector_timing and use it, if available
find_package(hector_timing QUIET)
if(hector_timing_FOUND)
  list(APPEND catkin_INCLUDE_DIRS ${hector_timing_INCLUDE_DIRS})
  list(APPEND catkin_LIBRARIES ${hector_timing_LIBRARIES})
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###########################
## Package configuration ##
###########################
set(HECTOR_POSE_ESTIMATION_MAXIMUM_STATE_VARIABLES 18 CACHE STRING "Maximum number of state variables supported by hector_pose_estimation. Used to preallocate memory for matrices and vectors. hector_pose_estimation_core itself uses a maximum of 18 variables.")
configure_file(include/hector_pose_estimation/matrix_config.h.in ${CATKIN_DEVEL_PREFIX}/include/hector_pose_estimation/matrix_config.h @ONLY)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES hector_pose_estimation
  CATKIN_DEPENDS rostime rosconsole roscpp sensor_msgs geometry_msgs nav_msgs geographic_msgs tf
)

###########
## Build ##
###########

## Explicitly set CMAKE_BUILD_TYPE and overwrite some compiler flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")
message(STATUS "CMAKE_BUILD_TYPE:               ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE:        ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "CMAKE_CXX_FLAGS_MINSIZEREL:     ${CMAKE_CXX_FLAGS_MINSIZEREL}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG:          ${CMAKE_CXX_FLAGS_DEBUG}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(hector_pose_estimation
  src/global_reference.cpp
  src/pose_estimation.cpp
  src/parameters.cpp
  src/types.cpp
  src/state.cpp

  src/filter.cpp
  src/filter/ekf.cpp

  src/system.cpp
  src/system/generic_quaternion_system_model.cpp
  src/system/ground_vehicle_model.cpp
  src/system/imu_model.cpp

  src/measurement.cpp
  src/measurements/gravity.cpp
  src/measurements/zerorate.cpp
  src/measurements/heading.cpp
  src/measurements/magnetic.cpp
  src/measurements/height.cpp
  src/measurements/baro.cpp
  src/measurements/gps.cpp
  src/measurements/poseupdate.cpp
  src/measurements/rate.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(hector_pose_estimation hector_worldmodel_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hector_pose_estimation
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS hector_pose_estimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/hector_pose_estimation ${CATKIN_DEVEL_PREFIX}/include/hector_pose_estimation
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
# )
