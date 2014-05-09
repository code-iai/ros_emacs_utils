
cmake_minimum_required(VERSION 2.8.3)
project(rosemacs)

find_package(catkin REQUIRED)
catkin_package()

# install(FILES swank.lisp repl-config.el
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})