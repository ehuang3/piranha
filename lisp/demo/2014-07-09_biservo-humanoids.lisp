(in-package :piranha)

;; (defun init-pose ()
;;   (pir-rotate :right +r-left)


(defun biservo-rel (tf &optional (q (quaternion (vec 0 0 0 0))))
  (pir-message "biservo-rel"
               (veccat (vec-array tf)
                       (vec-array q))))

;(pir-go-1 :left (tf (g* +r-right+  (x-angle  pi)) (vec .5 .1 0)))

(defun test-it ()
  (biservo-rel (tf (x-angle 0) (vec .05 0 0))
               (g* +r-right+  (x-angle  pi))))
