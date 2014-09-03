
cmake_minimum_required(VERSION 2.8.3)
project(roslisp_repl)

find_package(catkin REQUIRED)
catkin_package()

#############
## Compile ##
#############

configure_file(
  ${PROJECT_SOURCE_DIR}/sbclrc.in
  ${PROJECT_SOURCE_DIR}/sbclrc)
configure_file(
  ${PROJECT_SOURCE_DIR}/rosemacs-config.el.in
  ${PROJECT_SOURCE_DIR}/rosemacs-config.el)

#############
## Install ##
#############

install(FILES swank.lisp sbclrc
  general-config.el rosemacs-config.el slime-config.el repl-config.el
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS roslisp_repl
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
