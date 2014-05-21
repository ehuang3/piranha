(in-package :pir)

(defun demo-open ()
  (pir-pinch :right .14 .07))

(defun demo-close ()
  (pir-pinch :right .14 .04))

(defparameter +demo-r-down+
  (g* (g* +r-left+ (x-angle pi/2)) (y-angle (* -.5 pi))))

(defparameter +demo-r+
  (g* (g* +r-left+ (x-angle pi/2)) (y-angle (* -.25 pi))))

(defparameter +demo-offset+
  (quaternion-translation-2 (x-angle 0) (vec3 0 0.08 .0)))


(defvar *chan-reg*)
(defvar *chan-obj*)

(defun demo-chan ()
  (setq *chan-reg* (ach:open-channel "pir-reg"))
  (setq *chan-obj* (ach:open-channel "obj"))
)

(defvar *b-s-c*)
(defvar *c-s-o*)
(defvar *b-s-o*)

(defun demo-ready ()
  (pir-go-1 :right (quaternion-translation-2 +demo-r-down+ (vec3 .5 -.2 -.35))))

(defun demo-target ()
  (setq *b-s-c*
        (sns::msg-aref (sns::get-msg *chan-reg* 'sns::msg-tf)
                       1))

  (setq *c-s-o*
        (sns::msg-aref (sns::get-msg *chan-obj* 'sns::msg-tf)
                       0))
  ;(print *c-s-o*)
  (setq *c-s-o* (g*  *c-s-o* +demo-offset+))
  ;(print *c-s-o*)
  (setq *b-s-o*
        (g* *b-s-c* *c-s-o*)))

(defun demo-servo ()
  (pir-servo-cam *c-s-o* (quaternion-translation-2 +demo-r+ (vec3 0 0 0))))


(defun demo-approach ()
  (demo-target)
  (quaternion-translation-2 +demo-r+
                            (aa::g+ (vec3 0 -.08 .08)
                                    (aa::quaternion-translation-translation *b-s-o*))))


(defun demo-lift ()
  (let ((state (get-state)))
    (pir-go-1 :right (g* (quaternion-translation-2 (x-angle 0)
                                                   (vec3 0 0 .05))
                         (pir-state-e-f-r state))
              :time 5d0)))

(defun demo-run ()
  (pir-go-1 :right (demo-approach))
  (sleep 12)
  (demo-servo)
  (sleep 10)
  (demo-close)
  (sleep 2)
  (demo-lift))
