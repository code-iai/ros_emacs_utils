
cmake_minimum_required(VERSION 2.8.3)
project(slime_ros)

find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############

install(FILES slime-ros.el
  DESTINATION share/emacs/site-lisp)

install(FILES swank-ros.asd swank-ros.lisp
  DESTINATION share/emacs/common-lisp)
