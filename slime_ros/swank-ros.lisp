
(defpackage swank-ros
  (:use :cl)
  (:import-from :swank #:defslimefun)
  (:export #:load-ros-manifest))

(in-package :swank-ros)

(defmethod asdf:perform :around ((o asdf:load-op)
                                 (c asdf:cl-source-file))
  (handler-case (call-next-method o c)
    ;; If a fasl was stale, try to recompile and load (once).
    (sb-ext:invalid-fasl ()
      (asdf:perform (make-instance 'asdf:compile-op) c)
      (call-next-method))))

;;; Add appropriate paths for asdf to look for ros-load-manifest and load it
(defslimefun load-ros-manifest (asdf-system-directory)
  (unless (asdf:find-system :ros-load-manifest nil)
    (let ((load-manifest-directory
            (parse-namestring
             (concatenate 'string (namestring asdf-system-directory)
                          "/load-manifest/"))))
      (push load-manifest-directory asdf:*central-registry*)))
  (asdf:operate 'asdf:load-op :ros-load-manifest)
  (format t "~%ROS welcomes you!"))

;;; Redirect all the I/O from Swank SBCL process to standard I/O
(setf swank:*globally-redirect-io* t)

(provide :swank-ros)
