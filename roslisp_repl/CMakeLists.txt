
cmake_minimum_required(VERSION 2.8.3)
project(roslisp_repl)

find_package(catkin REQUIRED)
catkin_package()

#############
## Compile ##
#############

# file(READ "swank.lisp.txt" SWANK_LISP_TEXT)
# file(READ "sbclrc.txt" SBCLRC_TEXT)
# file(READ "repl-config.el.txt" REPL_CONFIG_TEXT)

# set(SWANK_LISP_TEXT_EDITED "")
# set(SBCLRC_TEXT_EDITED "")
# set(REPL_CONFIG_TEXT_EDITED "")
# set(SCRIPT_TEXT "
# #!/bin/bash

# ROSLISP_REPL_DIR=$(rospack find roslisp_repl)

# [ -f \${HOME}/.swank.lisp ] ||
# echo \"${SWANK_LISP_TEXT}\" > \${HOME}/bla/.swank.lisp

# [ -f \${HOME}/.sbclrc ] ||
# echo \"${SBCLRC_TEXT}\" > \${HOME}/bla/.sbclrc

# emacs -q -L \${ROSLISP_REPL_DIR} --eval \"(require 'repl-config)\"
# ")

# file(WRITE "repl-config.el" REPL_CONFIG_TEXT_EDITED)
# file(WRITE "roslisp_repl_test" ${SCRIPT_TEXT})

#############
## Install ##
#############

install(FILES swank.lisp sbclrc repl-config.el
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS roslisp_repl
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
