
cmake_minimum_required(VERSION 2.8.3)
project(slime_ros)

find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############

install(FILES slime-ros.el
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp/slime/contrib)

install(FILES swank-ros.asd swank-ros.lisp
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/common-lisp/source)
