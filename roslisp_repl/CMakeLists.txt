
cmake_minimum_required(VERSION 3.0.2)
project(roslisp_repl)

find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############

install(FILES repl-config.el
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS roslisp_repl
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
