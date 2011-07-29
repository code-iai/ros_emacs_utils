
(defun ros-package-path (pkg)
  (save-excursion
    (with-temp-buffer
      (call-process "rospack" nil t nil "find" pkg)
      (goto-char (point-min))
      (re-search-forward "^\\(.*\\)$")
      (match-string 1))))

(add-to-list 'load-path (ros-package-path "rosemacs"))
(add-to-list 'load-path (concat (ros-package-path "roslisp_repl") "/slime"))

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

(require 'slime)

(require 'rosemacs)
(invoke-rosemacs)
(global-set-key "\C-x\C-r" ros-keymap)

;; We need to fix the swank path in case someone instaled swank-cl and
;; didn't purge it again.
(setq slime-backend "swank-loader.lisp")

(slime-setup '(slime-fancy slime-asdf slime-indentation slime-ros))
(setq slime-complete-symbol-function 'slime-fuzzy-complete-symbol)

(setq slime-multiprocessing t)

(slime-ros)

(provide 'repl-config)
