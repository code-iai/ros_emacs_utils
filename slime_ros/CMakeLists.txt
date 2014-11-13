
cmake_minimum_required(VERSION 2.8.3)
project(slime_ros)

find_package(catkin REQUIRED)
catkin_package()

###############
## Configure ##
###############

configure_file(
  ${PROJECT_SOURCE_DIR}/sbclrc.in
  ${PROJECT_SOURCE_DIR}/sbclrc)
configure_file(
  ${PROJECT_SOURCE_DIR}/slime-config.el.in
  ${PROJECT_SOURCE_DIR}/slime-config.el)

#############
## Install ##
#############

install(FILES slime-config.el sbclrc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES slime-ros.el
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp/slime/contrib)

install(FILES swank-ros.asd swank-ros.lisp
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/common-lisp/source)

install(PROGRAMS slime_ros_init
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})