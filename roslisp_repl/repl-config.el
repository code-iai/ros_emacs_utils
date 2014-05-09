
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

(defun ros-package-path (pkg)
  (save-excursion
    (with-temp-buffer
      (call-process "rospack" nil t nil "find" pkg)
      (goto-char (point-min))
      (re-search-forward "^\\(.*\\)$")
      (match-string 1))))

(add-to-list 'load-path (ros-package-path "rosemacs"))
(require 'rosemacs)

(invoke-rosemacs)
(global-set-key "\C-x\C-r" ros-keymap)
(setq ros-completion-function (quote ido-completing-read))

;; slime

;; the following will go away, it's just needed for from source emacs installs
(add-to-list 'load-path "~/workspace/lisp/slime")

(require 'slime-autoloads)
(setq slime-backend "swank-loader.lisp")
(add-hook 'inferior-lisp-mode-hook (lambda () (inferior-slime-mode 1)))

(setq inferior-lisp-program "/usr/bin/sbcl --dynamic-space-size 4096")
(setq slime-lisp-implementations nil)

(add-to-list 'load-path (ros-package-path "slime_ros"))
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
