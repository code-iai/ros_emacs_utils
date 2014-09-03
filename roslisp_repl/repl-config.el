
;; general config

(customize-set-variable 'indent-tabs-mode nil)
(setq default-tab-width 2)
(global-font-lock-mode t)
(setq query-replace-highlight t)
(setq search-highlight t)
(show-paren-mode 1)
(global-set-key '[delete] 'delete-char)
(setq minibuffer-max-depth nil)
(setq auto-mode-alist
      (append '(("\\.C$"       . c++-mode)
                ("\\.cc$"      . c++-mode)
                ("\\.c$"       . c-mode)
                ("\\.h$"       . c++-mode)
                ("makefile$"   . makefile-mode)
                ("Makefile$"   . makefile-mode)
                ("\\.asd"      . lisp-mode)
                ("\\.launch"   . xml-mode)) auto-mode-alist))
(autoload 'mwheel-install "mwheel" "Enable mouse wheel support.")

;; rosemacs

(let ((default-directory "/home/gkazhoya/workspace/catkin/install/share/emacs/site-lisp"))
  (cond ((file-directory-p default-directory)
         (setq load-path
               (append
                (let ((load-path (copy-sequence load-path))) ;; Shadow
                  (append
                   (copy-sequence (normal-top-level-add-to-load-path '(".")))
                   (normal-top-level-add-subdirs-to-load-path)))
               load-path)))
        (t
         (message-box "Can't find the .el files!
Did you forget to install the ros_emacs_utils packages?
If so, run \"catkin_make install\" in your catkin workspace
or, if you prefer to only install the specific package,
run \"catkin_make install --pkg PACKAGE\" where PACKAGE is
rosemacs, slime, slime_ros and roslisp_repl."))))

(require 'rosemacs)

(invoke-rosemacs)
(global-set-key "\C-x\C-r" ros-keymap)
(setq ros-completion-function (quote ido-completing-read))

;; slime

(require 'slime-autoloads)
(setq slime-backend "swank-loader.lisp")
(add-hook 'inferior-lisp-mode-hook (lambda () (inferior-slime-mode 1)))

(setq inferior-lisp-program "/usr/bin/sbcl --dynamic-space-size 4096")
(setq slime-lisp-implementations nil)

(setq slime-contribs '(slime-repl
                       slime-autodoc
                       ;; slime-c-p-c
                       slime-editing-commands
                       slime-fancy-inspector
                       slime-fancy-trace
                       slime-fuzzy
                       slime-presentations
                       slime-scratch
                       slime-references
                       slime-package-fu
                       slime-fontifying-fu
                       slime-trace-dialog
                       ;;
                       slime-asdf
                       slime-indentation
                       slime-xref-browser
                       slime-highlight-edits
                       slime-ros))

(setq slime-complete-symbol-function 'slime-fuzzy-complete-symbol)

(slime)

(provide 'repl-config)
