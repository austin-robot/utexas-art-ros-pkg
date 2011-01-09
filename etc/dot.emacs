;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;	.emacs for robot on the car
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;	emacs version-specific initialization
;;;
;;;	This .emacs file is intended to handle both GNU emacs and
;;;	Lucid Xemacs, with major version numbers of 19, 20 or 21.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Add personal stuff to the standard load path.  It seems safer to
;; append these things to the end, but I want to override the standard
;; xemacs21 distribution's version of comint.el with a different
;; version.  **no longer doing that**
(setq load-path
      (cons (expand-file-name "~/lisp/") load-path))

;;; A few useful function keys
(global-set-key [f1]
		'(lambda () (interactive) (switch-to-buffer "*shell*")))
(global-set-key [f7] 'toggle-truncate-lines)
(global-set-key [f8] 'shell)

(setenv "PAGER" "/bin/cat")
(setenv "EDITOR" "/usr/bin/emacsclient")
;; start emacsclient server
(server-start)

;; some general settings:
(setq default-major-mode		'indented-text-mode
      dired-compression-method		'gzip
      find-file-visit-truename		t
      backup-by-copying-when-mismatch	t
      backup-by-copying-when-linked	t
      require-final-newline		t
      scroll-step			2)
(setq garbage-collection-messages	t
      gc-cons-threshold			4000000)
(setq efs-generate-anonymous-password	"joq@io.com")
(setq tex-dvi-print-command		"dvips"
      tex-dvi-view-command		"xdvi")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;	X window system initialization
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(if (eq window-system 'x)
    (progn
      ;;(global-set-key "\C-x\C-c" nil)	; avoid annoying typos
      ;;(global-set-key "\C-z" nil)	; (best to disable C-z with X)
      ;;(global-set-key "\C-x\C-z" nil)	; (disable C-x C-z, too)
      ;;(setq x-symbol-default-coding 'iso-8859-1)

      (if (eq 0 (user-uid))	; running as "root"?
          (progn (set-background-color "wheat")
                 (set-foreground-color "navyblue")))
      (transient-mark-mode t)

      (scroll-bar-mode 1)
      (setq scroll-preserve-screen-position t)
      ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;	ediff
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(autoload 'ediff-buffers "ediff" "Visual interface to diff" t)
(autoload 'ediff  "ediff"  "Visual interface to diff" t)
(autoload 'ediff-files "ediff" "Visual interface to diff" t)
(autoload 'epatch  "ediff"  "Visual interface to patch" t)
(autoload 'ediff-patch-file "ediff" "Visual interface to patch" t)
(autoload 'ediff-patch-buffer "ediff" "Visual interface to patch" t)
(autoload 'epatch-buffer "ediff" "Visual interface to patch" t)
(autoload 'vc-ediff "ediff"
  "Interface to diff & version control via vc.el" t) 
(autoload 'rcs-ediff "ediff"
  "Interface to diff & version control via rcs.el" t)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;	handle specific file types
;;;	(these go last so other stuff can't easily override them)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(add-hook 'text-mode-hook '(lambda () (auto-fill-mode 1)))
(add-hook 'python-mode-hook 'flyspell-prog-mode)
(add-hook 'c-mode-hook 'flyspell-prog-mode)
(add-hook 'c-mode-hook
	  '(lambda ()
	     (setq c-tab-always-indent	t
		   parens-require-spaces nil
		   comment-column 40)
	     (c-set-style "gnu")))	; for JAMin use "stroustrup"
(setq c++-mode-hook c-mode-hook)	; use same options for C++
(setq browse-url-new-window-p t)	; always use a new window for URLs
;;(setq find-file-hooks			; display full file name on modeline
;;      (cons '(lambda ()
;;	       (setq modeline-buffer-identification (list "%f")))
;;	    find-file-hooks))
(setq auto-mode-alist (cons '("\\.pl$" . perl-mode) auto-mode-alist))
(setq shell-multiple-shells t)		; allow multiple shell windows

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;	subversion
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(autoload 'psvn "svn-status" "Subversion command interface" t)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;	global key definitions, disabled functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define-key global-map "\C-c\C-e" 'ediff-buffers)
(define-key global-map "\C-c\C-p" 'bbdb-complete-name)
(define-key global-map "\M-\C-c" 'compile)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Robot Operating System tools -- requires environment settings
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(if (getenv "ROS_ROOT")
    (progn
      (add-to-list 'load-path (concat (getenv "ROS_ROOT") "/tools/rosemacs"))
      (require 'rosemacs)
      (invoke-rosemacs)
      (global-set-key "\C-x\C-r" ros-keymap)
      (add-to-list 'auto-mode-alist '("\\.bmr$" . python-mode))
      ))

(setq inhibit-startup-message t)
(message "Emacs personalization completed ")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;  End of personal settings --
;;;	Stuff from here on written by various versions of emacs...
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
