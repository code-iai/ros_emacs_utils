ros_emacs_utils
====================

This is a collection of packages to work on ROS-based software from Emacs.

Supported are the versions of Emacs >= 24.3. For older versions please use the
[older-than-emacs24.3 branch](https://github.com/code-iai/ros_emacs_utils/tree/older-than-emacs24.3).

The repo contains a ```rosemacs``` package, which provides functions for starting a roscore,
monitoring ROS nodes etc (with corresponding Emacs key bindings).
And, in addition to that, it has a number of packages to simplify
development of roslisp-based packages. Among them is
a wrapper for Slime (Lisp interactive development environment) called ```slime_wrapper```,
a contrib for Slime to work with ROS ```slime_ros```,
and a Slime REPL called ```roslisp_repl```, configured to start slime, slime_ros and setup roslisp.

This document only gives you instructions on installation.
For other information consult the official wiki pages of the packages:
[rosemacs](http://wiki.ros.org/rosemacs) for non Lisp programmers
and [roslisp_repl](http://wiki.ros.org/roslisp_repl) otherwise.


## Not a Common Lisp programmer

If you don't work with Common Lisp and just use Emacs for C++ or Python
or Java or Lisp dialects other than Common Lisp or whatever else,
you just need to add the following lines to your [Emacs initialization file](http://www.emacswiki.org/emacs/InitFile) (init.el or similar):

```lisp
(add-to-list 'load-path "/opt/ros/DISTRO/share/emacs/site-lisp")
(require 'rosemacs-config)
```
where ```DISTRO``` is the name of your ROS distribution, e.g. ```indigo```.

## Common Lisp programmer

### For users

If you work with roslisp, all you need to do is to start ```roslisp_repl``` in the terminal.

If you want to start the REPL from inside of your Emacs process, add the following to your Emacs init script:

```lisp
(require 'slime-config "PATH_TO_SLIME_ROS/slime-config.el")
```
where ```PATH_TO_SLIME_ROS``` is what ```rospack find slime_ros``` gives you, e.g. ```"/opt/ros/indigo/share/slime_ros"```, or ```"YOUR_CATKIN_WS/src/ros_emacs_utils/slime_ros"```
if you're installing from source. After that line you can add the usual Slime
customization commands, like setting the ```inferior-lisp-program``` or
turning off the ```slime-startup-animation``` etc.

Then you need to run
```bash
$ rosrun slime_ros slime_ros_init
```
which will create ```.sbclrc-ros``` in your home directory
and add an entry into your ```.sbclrs``` to load ```.sbclrc-ros```.

Once set up, you can start the REPL from your Emacs by pressing ```M-x slime```,
which means holding the ```Alt``` key and pressing ```x``` and then typing
```slime``` .

### For developers

There is one detail to take into account when **compiling ros_emacs_utils from source**:
in order for the code to work you not only need to run ```catkin_make``` on the packages,
but also install them (```catkin_make install```).

Why do we need to ```catkin_make install```? (Skip the next two paragraphs if you don't care.)

All the packages have their Emacs Lisp part contained in a single or multiple ```*.el``` files.
During installation of the packages those files are being copied
into ```YOUR_INSTALL_DIR/share/emacs/site-lisp```. Therefore, you need to tell Emacs
in the initialization script to add that directory to the Emacs ```load-path```
in a recursive way. That is done in ```rosemacs-config.el```.

In addition to the Emacs Lisp part, all the packages except ```rosemacs```
have a Common Lisp part, and all the ```*.lisp``` files are being copied
into ```YOUR_INSTALL_DIR/share/common-lisp/source```,
this replicates the Debian approach to installing Emacs Lisp and Common Lisp files.
Therefore, you need to tell your Common Lisp compiler, actually linker, i.e. ASDF,
to search for systems in that directory. That is done in ```.sbclrc```,
or, more correctly, in ```.sbclrc-ros```. As you can see, right now only SBCL is supported.
The original file can be found in your ```slime_ros``` ROS package under the name ```sbclrc-ros```.
When starting the ```roslisp_repl``` executable, ```slime_ros_init``` is called,
which in its turn copies ```sbclrc-ros``` into the home directory,
and adds a necessary entry into ```.sbclrc```.
Check the ```slime_ros_init``` executable from ```slime_ros``` package for more info.

## System requirements

* Emacs24.3+
* For Common Lisp developers: SBCL as the preferred compiler


## FAQ

* Q: Why doesn't my ```roslisp_repl``` start properly / find ```rosemacs```?
* A: Probably because you didn't install the ```ros_emacs_utils``` packages,
e.g. ```catkin_make install``` them.
Just follow the directions in the error pop up winodw (or echo buffer) of your Emacs.

-

* Q: It says component "swank" cannot be found.
* A: There might be something wrong in your ```.sbclrc``` or ```.sbclrc-ros```.
Try calling ```rosrun slime_ros slime_ros_init``` if you haven't done that yet.
(When starting REPL through ```roslisp_repl``` the script is called automatically.) 

-

* Q: It still doesn't work!
* A: Please file a bug report on Github.
