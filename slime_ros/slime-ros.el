(require 'slime)
(require 'rosemacs)

(define-slime-contrib slime-ros
	"Extension of slime for utilizing rosemacs features"
	(:authors "ROS Community")
	(:license "BSD")
	(:slime-dependencies slime-asdf)
	(:swank-dependencies swank-ros)
	(:on-load (add-hook 'slime-connected-hook 'slime-ros-load-manifest)))

(defcustom slime-ros-completion-function 'completing-read
  "The completion function to be used for package and system
  completions. This variable can be set to `ido-completing-read'
  to enable `ido-mode' for ros packages."
  :type 'function
  :group 'rosemacs)

(defvar slime-ros-package-history nil)

(defun slime-ros-load-manifest ()
	(let ((roslisp-path (ros-package-dir "roslisp")))
		(when roslisp-path
				(slime-eval-async `(swank-ros:load-ros-manifest ,roslisp-path)
					(lambda (result)
            (message "Successfully loaded ros-load-manifest."))))))

(defun slime-ros-read-pkg-name (&optional prompt default-value)
  (cond ((not (slime-current-connection))
          (message "Not connected."))
        (t
         (let ((default (slime-eval `(cl:identity ros-load:*current-ros-package*))))
           (ros-completing-read-package nil default slime-ros-completion-function)))))

(defun slime-ros-replace-underscores (str)
  (replace-regexp-in-string "_" "-" str))

(defun slime-ros-get-systems-in-pkg (package &optional default-value prompt)
  (let* ((package-path (ros-package-path package))
         (asd-files (append (ros-files-in-package package-path "asd" "asdf")
                            (ros-files-in-package package-path "asd" ".")))
         (default2 (slime-ros-replace-underscores default-value))
         (default (cond ((member default-value asd-files) default-value)
                        ((member default2 asd-files) default2)))
         (prompt (concat (or prompt (format "ROS Package `%s', System" package))
                         (if default
                             (format " (default `%s'): " default)
                           ": "))))
    (funcall slime-ros-completion-function
             prompt (mapcar #'car (slime-bogus-completion-alist asd-files))
             nil nil nil nil default)))

(defslime-repl-shortcut slime-repl-load-ros-system ("ros-load-system")
  (:handler (lambda ()
              (interactive)
              (let* ((ros-pkg-name (slime-ros-read-pkg-name))
                     (path (ros-package-path ros-pkg-name))
                     (system-name (slime-ros-get-systems-in-pkg ros-pkg-name ros-pkg-name)))
                (slime-cd path)
                (setq default-directory path)
                (slime-eval `(cl:setf ros-load:*current-ros-package* ,ros-pkg-name))
                (slime-oos system-name 'load-op)))))

(defslime-repl-shortcut slime-repl-load-ros-system ("ros-test-system")
  (:handler (lambda ()
              (interactive)
              (let* ((ros-pkg-name (slime-ros-read-pkg-name))
                     (system-name (slime-ros-get-systems-in-pkg ros-pkg-name ros-pkg-name)))
                (slime-eval `(cl:setf ros-load:*current-ros-package* ,ros-pkg-name))
                (slime-oos system-name 'test-op)))))

(provide 'slime-ros)
