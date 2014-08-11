cmake_minimum_required(VERSION 2.8.3)
project(slime)
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

install(FILES ${SLIME_EL_FILES}
  DESTINATION share/emacs/site-lisp/slime)
install(FILES ${SLIME_CONTRIB_EL_FILES}
  DESTINATION share/emacs/site-lisp/slime/contrib)
install(FILES ${SLIME_LIB_EL_FILES}
  DESTINATION share/emacs/site-lisp/slime/lib)
install(FILES ${SLIME_LISP_FILES}
  DESTINATION share/common-lisp/source/slime)
install(FILES ${SLIME_ASD_FILES}
  DESTINATION share/common-lisp/source/slime)
install(FILES ${SLIME_CONTRIB_LISP_FILES}
  DESTINATION share/common-lisp/source/slime/contrib)
install(FILES src/ChangeLog DESTINATION share/emacs/site-lisp/slime)
install(FILES src/ChangeLog DESTINATION share/common-lisp/source/slime)