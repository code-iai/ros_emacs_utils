
cmake_minimum_required(VERSION 2.8.3)
project(slime)

include(ExternalProject)
ExternalProject_Add(slime_cvs
  PREFIX slime
  CVS_REPOSITORY ":pserver:anonymous:anonymous@common-lisp.net:/project/slime/cvsroot"
  CVS_MODULE slime
  BUILD_IN_SOURCE
  # Do nothing. Slime doesn't need to build anything.
  CONFIGURE_COMMAND echo
  BUILD_COMMAND echo
  INSTALL_COMMAND echo)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/slime/src/slime_cvs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
