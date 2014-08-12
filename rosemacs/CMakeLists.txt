
cmake_minimum_required(VERSION 2.8.3)
project(rosemacs)

find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############

install(FILES rosemacs.el rosbag-view-mode.el
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp)

install(PROGRAMS poll-rosnode poll-rostopic
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp)

install(FILES manifest.rnc roslaunch.rnc rng-schemas.xml
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp)

# hmm, snippets should go into the home... but where exactly?... .emacs.d?...
# foreach(subdir
#     c++-mode)
#   install(DIRECTORY snippets/${subdir}
#     DESTINATION share/emacs/snippets)
# endforeach()
