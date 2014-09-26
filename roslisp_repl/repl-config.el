
;;; General configuration
(customize-set-variable 'indent-tabs-mode nil)
(setq default-tab-width 2)
(global-font-lock-mode t)
(setq query-replace-highlight t)
(setq search-highlight t)
(show-paren-mode 1)
(global-set-key '[delete] 'delete-char)
(setq minibuffer-max-depth nil)
(autoload 'mwheel-install "mwheel" "Enable mouse wheel support.")

;;; Start slime
;; ``slime-config`` is located in the ``slime_ros`` package.
;; It's path is passed to emacs through the -L argument of
;; the ``roslisp_repl`` executable.
(require 'slime-config)
(slime)

;;; Footer
(provide 'repl-config)
