cmake_minimum_required(VERSION 2.8.3)
project(slime_wrapper)
find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############

file(GLOB SLIME_EL_FILES "src/*.el")
file(GLOB SLIME_CONTRIB_EL_FILES "src/contrib/*.el")
file(GLOB SLIME_LIB_EL_FILES "src/lib/*.el")
file(GLOB SLIME_LISP_FILES "src/*.lisp")
file(GLOB SLIME_ASD_FILES "src/*.asd")
file(GLOB SLIME_CONTRIB_LISP_FILES "src/contrib/*.lisp")
file(GLOB SLIME_SWANK_LISP_FILES "src/swank/*.lisp")

install(FILES ${SLIME_EL_FILES}
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp/slime)
install(FILES ${SLIME_CONTRIB_EL_FILES}
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp/slime/contrib)
install(FILES ${SLIME_LIB_EL_FILES}
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp/slime/lib)
install(FILES ${SLIME_LISP_FILES}
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/common-lisp/source/slime)
install(FILES ${SLIME_ASD_FILES}
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/common-lisp/source/slime)
install(FILES ${SLIME_CONTRIB_LISP_FILES}
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/common-lisp/source/slime/contrib)
install(FILES ${SLIME_SWANK_LISP_FILES}
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/common-lisp/source/slime/swank)
install(FILES src/ChangeLog
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/emacs/site-lisp/slime)
install(FILES src/ChangeLog
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/common-lisp/source/slime)