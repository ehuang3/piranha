(in-package :pir)

(defvar *chan-obj*)
(defvar *cam-e-obj*)

(defparameter *body-e-cam-raw*
  (quaternion-translation
   ;; static calibration
   (aa:vec -0.728869 0.682359 -0.018334 0.052918 0.443670 0.100229 0.635026)))

(defparameter *body-e-cam*
  (aa::make-tf-tag
   :tf *body-e-cam-raw*
   :parent 'body
   :child 'camera))

(defparameter *cal-pt0-cam* (vec3 .430 .09 1.16))
(defparameter *cal-pt0-sdh* (vec3 0.5092758157695867d0 -0.3370171960730668d0
                               -0.41414430792945567d0))

(defvar *cam-e-obj*)


;(g* *body-e-cam* *cam-e-obj*)

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

;; (defun grasp-it (grasp &key (cam-e-obj *cam-e-obj*))
;;   (pir-sdh-set :right (simple-grasp-q0 grasp))
;;   (sleep 1)
;;   (let* ((sdh-pose (aa::tf-tag-tf (aa::g-chain  *body-e-cam* cam-e-obj (simple-grasp-pose grasp))))
;;          (sdh-approach (g* sdh-pose (aa::build-tf :x -.12))))
;;     (sdh-trajx (list (trajx-point sdh-approach 10d0)
;;                      (trajx-point sdh-pose 5d0))))
;;   (sleep 15)
;;   (pir-sdh-set :right (simple-grasp-q1 grasp))
;;   (sleep 2)
;;   (pir-go-rel :right :x -.1))


(defun cam-pose-kludge (cam-pose)
  (g* (aa::make-tf-tag :tf (aa::build-tf :z .00) :parent 'camera :child 'camera) cam-pose))

(defun obj-pose ()
  (let ((msg (sns:get-msg *chan-obj* 'sns:msg-tf :last t :wait t)))
    (setq *cam-e-obj*
          (aa::make-tf-tag :tf (sns:msg-aref msg 0)
                           :child 'object
                           :parent 'camera))))


(defvar *body-e-obj*)

(defun grasp-it (grasp) ;&key (cam-e-obj *cam-e-obj*))
  (pir-sdh-set :right (simple-grasp-q0 grasp))
  (sleep 1)
  (pir-go :right (list (trajx-point (g* *body-e-obj* (aa::build-tf :x -.12))  10d0)
                       (trajx-point *body-e-obj*  5d0))
          :point :finger)
  (sleep 15)
  (pir-sdh-set :right (simple-grasp-q1 grasp))
  (sleep 2)
  (pir-go-rel :right :x -.1))

(defun reverse-pose () (setq *body-e-obj* (pir-state-e-f-r (get-state))))



  ;; (let* ((state (get-state))
  ;;        (body-e-obj (aa::g-chain (pir-state-e-r state) ;body-e-wrist
  ;;                                 +wrist-e-sdh+  ;wrist-e-sdh
  ;;                                 (inverse (aa::tf-tag-tf (simple-grasp-pose grasp))))))
  ;;   (setq *cam-e-obj*
  ;;         (aa::make-tf-tag :tf (g* (inverse (aa::tf-tag-tf *body-e-cam*))
  ;;                                  body-e-obj)
  ;;                          :parent 'camera
  ;;                          :child 'object))))


;; (grasp-it *grasp-clutch-1*)
;; (pir-untuck :right)
