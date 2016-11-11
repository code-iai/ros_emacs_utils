
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
;; Enable copy-pasting between programs (Kill-ring <-> x11)
(setq x-select-enable-clipboard t)
(cond
 ((fboundp 'x-cut-buffer-or-selection-value)
  (setq interprogram-paste-function 'x-cut-buffer-or-selection-value))
 ((fboundp 'x-selection-value) ;; emacs 24 or later
  (setq interprogram-paste-function 'x-selection-value))
 (t (setq x-select-enable-clipboard nil))) ;; no connection to X server

;;; Start slime
;; ``slime-config`` is located in the ``slime_ros`` package.
;; It's path is passed to emacs through the -L argument of
;; the ``roslisp_repl`` executable.
(require 'slime-config)

;; The following gets rid of the gray highlighting of uncompiled code
;; that can be confusing and annoying for the beginner Lispers.
(add-hook 'slime-mode-hook (lambda () (slime-highlight-edits-mode 0)))

;; some key bindings to, e.g., enable easy autocompletion etc.
(eval-after-load 'slime
  '(progn
     ;; Fix for M-, when using it with dired and A
     (define-key slime-mode-map (kbd "M-,")
       (lambda ()
         (interactive)
         (condition-case nil
             (slime-pop-find-definition-stack)
           (error (tags-loop-continue)))))
     (global-set-key "\C-cs" 'slime-selector)
     (define-key slime-repl-mode-map (kbd "C-M-<backspace>")
       'slime-repl-delete-current-input)
     (define-key slime-mode-map "\r" 'newline-and-indent)
     (define-key slime-mode-map [tab]
       (lambda ()
         (interactive)
         (let ((yas-fallback-behavior nil))
           (unless (yas-expand)
             (slime-fuzzy-indent-and-complete-symbol)))))
     (define-key slime-mode-map (kbd "M-a")
       (lambda ()
         (interactive)
         (let ((ppss (syntax-ppss)))
           (if (nth 3 ppss)
               (goto-char (1+ (nth 8 ppss)))
             (progn
               (backward-up-list 1)
               (down-list 1))))))
     (define-key slime-mode-map (kbd "M-e")
       (lambda ()
         (interactive)
         (let ((ppss (syntax-ppss)))
           (if (nth 3 ppss)
               (progn
                 (goto-char (nth 8 ppss))
                 (forward-sexp 1)
                 (backward-char 1))
             (progn
               (up-list 1)
               (backward-down-list 1))))))))

;;; [ and ] should be handled paranthesis-like in lisp files.
(modify-syntax-entry ?\[ "(]  " lisp-mode-syntax-table)
(modify-syntax-entry ?\] ")[  " lisp-mode-syntax-table)

(slime)

(delete-other-windows)

;;; Footer
(provide 'repl-config)
