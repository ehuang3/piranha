(in-package :pir)

(defun demo-open ()
  (pir-pinch :right .14 .06))

(defun demo-close ()
  (pir-pinch :right .14 .04))

(defparameter +demo-r-down+
  (g* (g* +r-left+ (x-angle pi/2)) (y-angle (* -.5 pi))))

(defparameter +demo-r+
  (g* (g* +r-left+ (x-angle pi/2)) (y-angle (* -.25 pi))))

(defparameter +demo-offset-obj+
  ;(quaternion-translation-2 (x-angle 0) (vec3 0 0.04 .0)))
  (quaternion-translation-2 (x-angle 0) (vec3 .00 0.00 .0)))


(defparameter +demo-offset-global+ (vec3 .00 0.00 -.04))

(defvar *chan-reg*)
(defvar *chan-obj*)
(defvar *chan-marker*)

(defun demo-chan ()
  (setq *chan-reg* (ach:open-channel "pir-reg"))
  (setq *chan-obj* (ach:open-channel "obj"))
  (setq *chan-marker* (ach::open-channel "markers"))
)

(defun get-marker (i)
  (sns::msg-aref (sns::get-msg *chan-marker* 'sns::msg-wt-tf) i))

(defvar *b-s-c*)
(defvar *c-s-o*)
(defvar *b-s-o*)

(defparameter +demo-ready+
  (quaternion-translation-2 +demo-r-down+ (vec3 .5 -.15 -.35)))

(defun demo-ready ()
  (pir-go-1 :right +demo-ready+))

(defun demo-target ()
  (setq *b-s-c*
        (sns::msg-aref (sns::get-msg *chan-reg* 'sns::msg-tf)
                       1))

  (setq *c-s-o*
        (sns::msg-aref (sns::get-msg *chan-obj* 'sns::msg-tf)
                       0))

  ;(setq *c-s-o* (get-marker 9))

  ;(print *c-s-o*)
  (setq *c-s-o* (g*  *c-s-o* +demo-offset-obj+))
  ;(print *c-s-o*)
  (setq *b-s-o*
        (g* (quaternion-translation-2 (x-angle 0) +demo-offset-global+)
            (g* *b-s-c* *c-s-o*)))
  )

(defun demo-servo ()
  (pir-servo-cam *c-s-o*
                 (quaternion-translation-2 +demo-r+ +demo-offset-global+)))


(defun demo-approach ()
  (demo-target)
  (g* (quaternion-translation-2 +demo-r+
                                (aa::quaternion-translation-translation *b-s-o*))
      (quaternion-translation-2 (x-angle 0)
                                (vec3 -.1 0 0))))

                            ;; (aa::g+ (vec3 0 -.08 .08)
                            ;;         (aa::quaternion-translation-translation *b-s-o*))))

(defparameter +demo-lift-height+ .05)

(defun demo-lift ()
  (let ((state (get-state)))
    (pir-go-1 :right (g* (quaternion-translation-2 (x-angle 0)
                                                   (vec3 0 0 +demo-lift-height+))
                         (pir-state-e-f-r state))
              :time 3d0)))


(defun demo-down ()
  (let ((state (get-state)))
    (pir-go-1 :right (g* (quaternion-translation-2 (x-angle 0)
                                                   (vec3 0 0 (- +demo-lift-height+)))
                         (pir-state-e-f-r state))
              :time 3d0)))

(defun demo-drop ()
  (demo-down)
  (sleep 4)
  (demo-open)
  (sleep 1)
  (let ((state (get-state)))
    (pir-go :right (list (trajx-point (g* (quaternion-translation-2 (x-angle 0)
                                                                    (vec3 0 0 .12))
                                          (pir-state-e-f-r state))
                                      3.5d0)
                         (trajx-point +demo-ready+
                                      5d0)))))

(defun demo-run (&optional (t0 5d0))
  (demo-open)
  (sleep 1)
  (pir-go-1 :right (demo-approach) :time t0)
  (sleep (+ 1 t0))
  (demo-servo)
  (sleep 6)
  (demo-close)
  (sleep 2)
  (demo-lift)
  (sleep 5d0)
  (demo-drop)
  )
