cmake_minimum_required(VERSION 3.0.2)
project(rtt_hector_pose_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS hector_pose_estimation_core)

## Find Orocos RTT
set(OROCOS_TARGET gnulinux)
find_package(OROCOS-RTT QUIET)
if(NOT OROCOS-RTT_FOUND)
  message(WARNING "Orocos RTT was not found. Package ${PROJECT_NAME} will not be built.")
  return()
else()
  include(${OROCOS-RTT_USE_FILE})
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS hector_pose_estimation_core
    DEPENDS
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

orocos_component(rtt_hector_pose_estimation
  src/taskcontext.cpp
  src/services.cpp
  src/parameters.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(rtt_hector_pose_estimation
  ${catkin_LIBRARIES}
  ${USE_OROCOS_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS rtt_hector_pose_estimation
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/hector_pose_estimation
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
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

####################
## Orocos Package ##
####################

orocos_generate_package(
  DEPENDS_TARGETS rtt_rosnode rtt_sensor_msgs rtt_geometry_msgs rtt_nav_msgs
)
