
cmake_minimum_required(VERSION 2.8.3)
project(roslisp_repl)

find_package(catkin REQUIRED)
catkin_package()

install(FILES swank.lisp sbclrc repl-config.el
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS roslisp_repl
  DESTINATION bin)
