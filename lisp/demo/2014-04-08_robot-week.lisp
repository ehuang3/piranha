;;;; COMMAND REFERENCE
;;;; =================
;;;;
;;;; Open Channels
;;;; -------------
;;;; (demo-init)
;;;;
;;;; Tuck Arm
;;;; ---------
;;;; (pir-tuck :right)
;;;;
;;;; Read Object Pose
;;;; ----------------
;;;; (obj-pose)
;;;;
;;;; Untuck Arm
;;;; ---------
;;;; (pir-untuck :right)
;;;;
;;;; Grasp Hinge
;;;; -----------
;;;; (grasp-it *grasp-hinge*)
;;;;
;;;; Go to initial position (joint trajectory)
;;;; -----------------------------------------
;;;; (pir-set :right *q-go-r*)
;;;;
;;;; Wave
;;;; ---------
;;;; ; from start position
;;;; (pir-set :right +q-r-wave-0+) ; wave initial position
;;;; (wave) ; wave trajectory
;;;; (pir-set :right *q-go-r*) ; return to start position




(in-package :pir)

(defparameter +q-r-wave-0+
  (vec 0.6775193623316788d0 1.3165716679494026d0 -1.3637130110457694d0
       1.2390615958683344d0 2.771129066561477d0 1.0862754731487507d0
       -2.205991454765713d0))

(defun wave ()
  (let* ((state (get-state))
         (e-0 (pir-state-e-r state))
         (e-r (quaternion-translation-2 (quaternion (y-angle (degrees 30)))
                                        (vec3 0 0 0)))
         (e-1 (g* e-0 e-r))
         (e-2 (g* e-0 (inverse e-r))))
    (pir-go :right (list (trajx-point e-1 2.1d0)
                         (trajx-point e-2 2.5d0)
                         (trajx-point e-0 2.1d0))
            :point :wrist)))


(defvar *chan-obj*)
(defvar *cam-e-obj*)

(defun demo-init ()
  (pir-start)
  (setq *chan-obj* (ach:open-channel "obj")))

(defparameter *body-e-cam-raw*
  (quaternion-translation
   (aa:vec -0.734447 0.677851 -0.000253 0.033253 0.420708 0.098608 0.659014)))

(defparameter *body-e-cam*
  (aa::make-tf-tag
   :tf *body-e-cam-raw*
   :parent 'body
   :child 'camera))

(defparameter *cal-pt0-cam* (vec3 .430 .09 1.16))
(defparameter *cal-pt0-sdh* (vec3 0.5092758157695867d0 -0.3370171960730668d0
                               -0.41414430792945567d0))

(defvar *cam-e-obj*)

(defstruct simple-grasp
  pose
  q0
  q1)

(defun kludge-close (q0 q1 q2 q3 q4 q5 q6)
  (let ((factor-1 1.00)
        (factor-2 1.00))
    (vec q0
         (* factor-1 q1)  (* factor-2 q2)
         q3 q4
         (* factor-1 q5)  (* factor-2 q6))))

(defun kludge-open (q0 q1 q2 q3 q4 q5 q6)
  (let ((factor-1 1.00)
        (factor-2 1.00))
    (vec q0
         (* factor-1 q1)  (* factor-2 q2)
         q3 q4
         (* factor-1 q5)  (* factor-2 q6))))

(defun ana-grasp (pose q0 q1)
  (make-simple-grasp :pose
                     (aa::make-tf-tag :tf (quaternion-translation (apply #'vec pose))
                                      :parent 'object
                                      :child 'sdh)
                     :q0 (apply #'kludge-open q0)
                     :q1 (apply #'kludge-close q1)))



(defparameter *grasp-hinge*
  (ana-grasp
   '(-0.403476 -0.237993 -0.701733 0.536784 0.0112289 0.076713 -0.0822397 )
   '(1.5708 -0.408054 0.784835 -1.5708 -1.5708 -0.408054 0.784835)
   '(1.5708 -0.396179 0.79671 -1.5708 -1.5708 -0.380867 0.812023)))

(defparameter *grasp-hinge-2*
  (ana-grasp
   '(-0.0911991 0.0571066 -0.109 0.82394 0.350305 -0.445356 -0.00826581)
   '(0.814484 -0.473742 0.796809 -0.473742 0.796809 -0.473742 0.796809)
   '(0.814484 -0.359367 0.911184 -0.443117 0.827434 -0.386555 0.883996)))

(defparameter *grasp-clutch-1*
  (ana-grasp
   '(0.365734 -0.425204 0.791525 -0.242753 -0.0208989 0.0689481 -0.105768 )
   '(1.5708 -0.404177 0.786161 -1.5708 -1.5708 -0.404177 0.786161)
   '(1.5708 -0.261989 0.928349 -1.5708 -1.5708 -0.286677 0.903661)))

(defun pir-go-cam (side x)
  (pir-go-1 side (quaternion-translation-2 +r-down+ (aa:transform *body-e-cam* x))))


(defun go-obj-test ()
  (let ((xyz (aa::quaternion-translation-translation (aa::tf-tag-tf (g* *body-e-cam* *cam-e-obj*)))))
    (pir-go-1 :right (quaternion-translation-2 +r-down+
                                               (aa::g+ xyz (vec3 0 0 .05))))))

(defun sdh-trajx-1 (b-e-sdh &key (time 10d0))
  (let ((b-e-w (g* b-e-sdh (aa::inverse +wrist-e-sdh+))))
    (pir-go-1 :right b-e-w :point :wrist :time time)))

(defun sdh-trajx (points)
  (let ((points (loop for p in points
                   collect
                     (trajx-point (g* (trajx-point-pose p) (aa::inverse +wrist-e-sdh+))
                                  (trajx-point-time p)))))
    (pir-go :right points :point :wrist)))

(defun grasp-target (grasp &key (cam-e-obj *cam-e-obj*))
  (aa::g-chain  *body-e-cam* cam-e-obj (simple-grasp-pose grasp)))


(defun obj-pose ()
  (let ((msg (sns:get-msg *chan-obj* 'sns:msg-tf :last t :wait t)))
    (setq *cam-e-obj*
          (aa::make-tf-tag :tf (sns:msg-aref msg 0)
                           :child 'object
                           :parent 'camera))))

(defun grasp-it (grasp &key (cam-e-obj *cam-e-obj*))
  ;(pir-sdh-set :right (simple-grasp-q0 grasp))
  (pir-pinch :right .14 .05)
  (sleep 1)
  (let* ((sdh-pose (aa::tf-tag-tf (aa::g-chain  *body-e-cam* cam-e-obj (simple-grasp-pose grasp))))
         (sdh-approach (g* sdh-pose (aa::build-tf :x -.12))))
    (sdh-trajx (list (trajx-point sdh-approach 7d0)
                     (trajx-point sdh-pose 3d0))))
  (sleep 10)
  ;(pir-sdh-set :right (simple-grasp-q1 grasp))
  (pir-pinch :right .14 .00)
  (sleep 2)
  (pir-go-rel :right :x -.1 :time 3d0))
