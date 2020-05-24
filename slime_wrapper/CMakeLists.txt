cmake_minimum_required(VERSION 3.0.2)
project(slime_wrapper)
find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############

install(DIRECTORY slime
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp)
