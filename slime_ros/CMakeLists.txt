
cmake_minimum_required(VERSION 3.0.2)
project(slime_ros)

find_package(catkin REQUIRED)
catkin_package()

###############
## Configure ##
###############

configure_file(
  ${PROJECT_SOURCE_DIR}/sbclrc-ros.in
  ${PROJECT_SOURCE_DIR}/sbclrc-ros)
configure_file(
  ${PROJECT_SOURCE_DIR}/slime-config.el.in
  ${PROJECT_SOURCE_DIR}/slime-config.el)

#############
## Install ##
#############

install(FILES slime-config.el sbclrc-ros
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS slime_ros_init
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
