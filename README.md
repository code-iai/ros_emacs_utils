ros_emacs_utils
====================

This is a collection of packages to work on ROS-based software from Emacs.

It has a ```rosemacs``` package, which contains functions for starting a roscore,
monitoring ROS nodes etc (with corresponding Emacs key bindings).
And, in addition to that, it has a number of packages to simplify
development of roslisp-based packages. Among them is
a wrapper for Slime (Lisp interactive development environment) called ```slime_wrapper```,
a contrib for Slime to work with ROS ```slime_ros```,
and a Slime REPL called ```roslisp_repl```, configured to start slime, slime_ros and roslisp.


## Not a Lisp programmer

If you don't work with Lisp and just use Emacs for C++ or Python or Java or whatever else,
you just need to add the following lines to your Emacs initialization file (init.el or similar):

```lisp
(add-to-list 'load-path "/opt/ros/DISTRO/share/emacs/site-lisp")
(require 'rosemacs)
(invoke-rosemacs)
(global-set-key "\C-x\C-r" ros-keymap)
(setq ros-completion-function (quote ido-completing-read))
```

Make sure to update ```DISTRO``` in the path.

## Lisp programmer

### For users

If you work with roslisp, all you need to do is to start ```roslisp_repl``` in the terminal.
If you want to start the REPL from inside of your Emacs process, copy the configuration
from the ```repl-config.el``` into your Emacs init script.
It can be found in your ```roslisp_repl``` ROS package.

### For developers

There is a number of things to take into account when compiling ros_emacs_utils from source.
In order for the code to work you not only need to run ```catkin_make``` on the packages,
but also install them (```catkin_make install```).

Why is that so?

All the packages have their Emacs Lisp part contained in a single or multiple ```*.el``` files.
During installation of the packages those files are being copied
into ```YOUR_INSTALL_DIR/share/emacs/site-lisp```. Therefore, you need to tell Emacs
in the initialization script to add that directory to the Emacs ```load-path```
in a recursive way. That is done in ```repl-config.el```.

In addition to the Emacs Lisp part, all the packages except ```rosemacs```
have a Common Lisp part, and all the ```*.lisp``` files are being copied
into ```YOUR_INSTALL_DIR/share/common-lisp/source```,
this replicates the Debian approach to installing Emacs Lisp and Common Lisp files.
Therefore, you need to tell your Common Lisp compiler, actually linker, i.e. ASDF,
to search for systems in that directory. That is done in ```.sbclrc```.
As you can see, right now only SBCL is supported.
The original file can be found in your ```roslisp_repl``` ROS package under the name ```sbclrc```.
When starting the ```roslisp_repl``` executive, this file is being copied into the home directory,
unless it already exists there. Check the ```roslisp_repl``` executive for more info.

### System requirements

This is only for the Lisp developers.
For non-Lisp developers things should be quite portable.

* Emacs24
* roslisp installed
* SBCL as the default Common Lisp compiler


### FAQ

* Q: Why doesn't my ```roslisp_repl``` start properly / find ```rosemacs```?
* A: Probably because you didn't install the ```ros_emacs_utils``` packages,
e.g. ```catkin_make install``` them.
Just follow the directions in the error pop up winodw (or echo buffer) of your Emacs.


* Q: I installed the packages. Why doesn't it still work?
* A: Please file a bug report on Github.
