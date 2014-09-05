
cmake_minimum_required(VERSION 2.8.3)
project(rosemacs)

find_package(catkin REQUIRED)
catkin_package()

###############
## Configure ##
###############

configure_file(
  ${PROJECT_SOURCE_DIR}/rosemacs-config.el.in
  ${PROJECT_SOURCE_DIR}/rosemacs-config.el)

#############
## Install ##
#############

install(FILES rosemacs.el rosbag-view-mode.el rosemacs-config.el
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp)

install(PROGRAMS poll-rosnode poll-rostopic
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp)

install(FILES manifest.rnc roslaunch.rnc rng-schemas.xml
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp)

foreach(subdir
    c++-mode)
  install(DIRECTORY snippets/${subdir}
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp/snippets)
endforeach()
install(FILES yasnippet.el DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp)
